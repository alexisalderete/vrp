from flask import Flask, request, jsonify, send_from_directory  
from pyvrp import Model
from pyvrp.stop import MaxRuntime
import requests
from flask_cors import CORS
import os
import math

app = Flask(__name__)
CORS(app)

# Ruta para servir la página principal
@app.route('/')
def serve_index():
    return send_from_directory(os.path.dirname(__file__), 'vrp_app.html')

OSRM_URL = "http://router.project-osrm.org/table/v1/driving/"

def get_matrices(coords):
    """Consulta OSRM para obtener matriz de distancias y tiempos"""
    try:
        coords_str = ";".join([f"{lon},{lat}" for lon, lat in coords])
        url = f"{OSRM_URL}{coords_str}?annotations=distance,duration"
        print(f"Consultando OSRM: {url}")
        
        res = requests.get(url, timeout=30)
        res.raise_for_status()
        data = res.json()
        
        print(f"Matriz de distancias: {data['distances']}")
        print(f"Matriz de duraciones: {data['durations']}")
        
        return data["distances"], data["durations"]
    except Exception as e:
        print(f"Error en OSRM: {e}")
        # Crear matriz de distancia euclidiana como fallback.
        size = len(coords)
        distances = [[0] * size for _ in range(size)]
        durations = [[0] * size for _ in range(size)]
        
        for i in range(size):
            for j in range(size):
                if i != j:
                    # Distancia euclidiana aproximada (1 grado ≈ 111 km)
                    lat1, lon1 = coords[i][1], coords[i][0]
                    lat2, lon2 = coords[j][1], coords[j][0]
                    dist = math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2) * 111000  # metros
                    distances[i][j] = int(dist)
                    durations[i][j] = int(dist / 13.9)  # tiempo en segundos (50 km/h)
        
        return distances, durations

def get_route_geometry(puntos):
    """
    Consulta OSRM para obtener la geometría tramo por tramo.
    Corrige automáticamente el orden lat/lon.
    """
    geometry = []

    for i in range(len(puntos) - 1):
        try:
            # Asegurar orden correcto: (lon, lat)
            lon1, lat1 = puntos[i][0], puntos[i][1]
            lon2, lat2 = puntos[i + 1][0], puntos[i + 1][1]

            coord_pair = f"{lon1},{lat1};{lon2},{lat2}"
            url = f"http://router.project-osrm.org/route/v1/driving/{coord_pair}?overview=full&geometries=geojson"

            res = requests.get(url, timeout=15).json()

            if res.get("routes") and len(res["routes"]) > 0:
                segment = res["routes"][0]["geometry"]["coordinates"]
                # Evitar duplicar el punto inicial del siguiente tramo
                if i > 0 and len(segment) > 1:
                    segment = segment[1:]
                geometry.extend(segment)
            else:
                print(f"⚠️ OSRM devolvió sin rutas para tramo {i}")
        except Exception as e:
            print(f"Error tramo {i}: {e}")
            continue

    return geometry
     

@app.route("/solve", methods=["POST"])
def solve():
    try:
        data = request.get_json()
        
        # Datos de vehículos
        num_vehicles = data.get("num_vehicles", 3)
        vehicle_capacity = data.get("vehicle_capacity", 1000)
        
        # Datos de pedidos
        orders = data.get("orders", [])
        coords = [order["coordinates"] for order in orders]
        weights = [order.get("weight", 1) for order in orders]
        time_windows = [order.get("time_window", [0, 1440]) for order in orders]
        
        # ✅ NUEVA VALIDACIÓN: Calcular número óptimo de vehículos
        total_demand = sum(weights[1:])
        optimal_vehicles = math.ceil(total_demand / vehicle_capacity)
        
        print(f"🔍 Análisis de optimización:")
        print(f"   - Demanda total: {total_demand} kg")
        print(f"   - Capacidad por vehículo: {vehicle_capacity} kg") 
        print(f"   - Vehículos solicitados: {num_vehicles}")
        print(f"   - Vehículos óptimos teóricos: {optimal_vehicles}")
        
        # ✅ FORZAR OPTIMIZACIÓN: Usar el mínimo necesario
        if num_vehicles > optimal_vehicles:
            print(f"   🔄 OPTIMIZANDO: Usando {optimal_vehicles} vehículos en lugar de {num_vehicles}")
            num_vehicles = optimal_vehicles
        
        # Validaciones
        if len(coords) < 2:
            return jsonify({"error": "Se necesitan al menos 2 puntos (depósito + clientes)"}), 400
        
        # Obtener matrices de OSRM
        dist_matrix, dur_matrix = get_matrices(coords)
        
        # Crear modelo VRP
        m = Model()
        
        # ✅ AGREGAR SOLO EL NÚMERO ÓPTIMO DE VEHÍCULOS
        vehicle_type = m.add_vehicle_type(
            num_available=num_vehicles,  # Ahora será 2 en lugar de 3
            capacity=vehicle_capacity
        )

        # El primer punto es el depósito
        depot = m.add_depot(
            x=coords[0][0], 
            y=coords[0][1],
            tw_early=0,
            tw_late=1440
        )

        # Agregar clientes
        clients = []
        for idx, (lon, lat) in enumerate(coords[1:]):
            weight = weights[idx + 1]
            tw_early, tw_late = time_windows[idx + 1]
            
            client = m.add_client(
                x=lon, 
                y=lat,
                delivery=weight,
                tw_early=tw_early,
                tw_late=tw_late,
                service_duration=15  # 15 minutos de servicio
            )
            clients.append(client)

        # Agregar aristas al modelo
        for i, frm in enumerate(m.locations):
            for j, to in enumerate(m.locations):
                if i != j:
                    # PyVRP espera distancias en metros y duraciones en segundos
                    distance = int(dist_matrix[i][j])
                    duration = int(dur_matrix[i][j])
                    m.add_edge(frm, to, distance=distance, duration=duration)

        # Resolver con timeout más corto
        stopping_criterion = MaxRuntime(30)
        
        res = m.solve(stop=stopping_criterion, display=True, seed=42)

        if not res.is_feasible():
            # Si no es factible con vehículos óptimos, intentar con 1 más
            if num_vehicles == optimal_vehicles:
                print(f"   🔄 Intentando con {optimal_vehicles + 1} vehículos...")
                num_vehicles = optimal_vehicles + 1
                
                # Recrear modelo con un vehículo adicional
                m = Model()
                vehicle_type = m.add_vehicle_type(
                    num_available=num_vehicles,
                    capacity=vehicle_capacity
                )
                depot = m.add_depot(x=coords[0][0], y=coords[0][1], tw_early=0, tw_late=1440)
                
                for idx, (lon, lat) in enumerate(coords[1:]):
                    weight = weights[idx + 1]
                    tw_early, tw_late = time_windows[idx + 1]
                    client = m.add_client(
                        x=lon, y=lat, delivery=weight,
                        tw_early=tw_early, tw_late=tw_late, service_duration=15
                    )
                
                for i, frm in enumerate(m.locations):
                    for j, to in enumerate(m.locations):
                        if i != j:
                            distance = int(dist_matrix[i][j])
                            duration = int(dur_matrix[i][j])
                            m.add_edge(frm, to, distance=distance, duration=duration)
                
                res = m.solve(stop=stopping_criterion, display=True, seed=42)
            
            if not res.is_feasible():
                return jsonify({
                    "error": "No se encontró una solución factible",
                    "details": "Intenta con menos clientes o mayor capacidad de vehículos"
                }), 400

        # PROCESAR RUTAS COMPLETAS (CON DEPÓSITO AL INICIO Y FINAL)
        routes = []
        total_distance = 0
        total_duration = 0
        used_vehicles = 0

        print(f"Número total de rutas en solución: {len(res.best.routes())}")

        for vehicle_id, route in enumerate(res.best.routes()):
            print(f"Ruta original {vehicle_id}: {list(route)}")
            
            # CORREGIR: AGREGAR DEPÓSITO AL INICIO Y FINAL SI NO ESTÁ PRESENTE
            route_indices = list(route)
            
            # Si la ruta no empieza con el depósito (índice 0), agregarlo
            if route_indices[0] != 0:
                route_indices.insert(0, 0)
                print(f"  → Agregado depósito al inicio: {route_indices}")
            
            # Si la ruta no termina con el depósito (índice 0), agregarlo
            if route_indices[-1] != 0:
                route_indices.append(0)
                print(f"  → Agregado depósito al final: {route_indices}")
            
            # Crear lista de coordenadas COMPLETA (incluyendo depósito)
            route_coords = [coords[idx] for idx in route_indices]
            
            # Puntos de entrega (excluyendo depósitos)
            delivery_points = []
            route_weights = []
            route_times = []
            
            # Calcular tiempos de llegada REALES
            current_time = 0  # tiempo desde salida del depósito (minutos)
            
            for idx, node_idx in enumerate(route_indices):
                if node_idx != 0:  # No es el depósito
                    delivery_points.append(coords[node_idx])
                    route_weights.append(weights[node_idx])
                    
                    # Tiempo de llegada
                    arrival_minutes = current_time
                    route_times.append(arrival_minutes)
                    
                    # Tiempo de servicio
                    current_time += 15
                
                # Agregar tiempo de viaje al siguiente punto (si existe)
                if idx < len(route_indices) - 1:
                    next_node_idx = route_indices[idx + 1]
                    travel_time_seconds = dur_matrix[node_idx][next_node_idx]
                    travel_time_minutes = travel_time_seconds / 60.0
                    current_time += travel_time_minutes
            
            # CALCULAR DISTANCIA Y DURACIÓN DE LA RUTA COMPLETA
            route_dist = 0
            route_dur = 0
            for i in range(len(route_indices)-1):
                route_dist += dist_matrix[route_indices[i]][route_indices[i+1]]
                route_dur += dur_matrix[route_indices[i]][route_indices[i+1]]
            
            # OBTENER GEOMETRÍA PARA LA RUTA COMPLETA
            geometry = []
            if len(route_coords) > 1:
                geometry = get_route_geometry(route_coords)
                print(f"Vehículo {vehicle_id+1}: {len(delivery_points)} clientes, {len(geometry)} puntos de geometría")
            else:
                print(f"Vehículo {vehicle_id+1}: Ruta vacía o con solo depósito")

            # INCLUIR RUTA EN LA RESPUESTA
            route_data = {
                "vehicle_id": vehicle_id + 1,
                "nodes": delivery_points,
                "weights": route_weights,
                "arrival_times": route_times,
                "full_route": route_coords,
                "geometry": geometry,
                "distance": route_dist,
                "duration": route_dur,
                "total_weight": sum(route_weights),
                "num_clients": len(delivery_points),
                "route_indices": route_indices  # Para depuración
            }
            
            routes.append(route_data)
            
            if len(delivery_points) > 0:
                used_vehicles += 1
                total_distance += route_dist
                total_duration += route_dur

                print(f"Ruta {vehicle_id+1}: {len(delivery_points)} clientes, "
                      f"distancia: {route_dist}m, duración: {route_dur}s")
                print(f"  Secuencia: {route_indices}")
            else:
                print(f"Ruta {vehicle_id+1}: Sin clientes asignados")

        print(f"Total de rutas procesadas: {len(routes)}")
        print(f"Vehículos usados: {used_vehicles}")

        return jsonify({
            "num_routes": len(routes),
            "routes": routes,
            "depot": coords[0],
            "vehicle_info": {
                "available": num_vehicles,
                "used": used_vehicles,
                "capacity": vehicle_capacity
            },
            "statistics": {
                "total_distance": total_distance,
                "total_duration": total_duration,
                "total_weight": sum(weights[1:]),
                "num_clients": len(clients)
            },
            "solution_quality": {
                "cost": float(res.cost()),
                "iterations": res.num_iterations,
                "is_feasible": res.is_feasible()
            }
        })

    except Exception as e:
        print(f"Error detallado en solve: {e}")
        import traceback
        print(f"Traceback completo: {traceback.format_exc()}")
        
        return jsonify({
            "error": "Error al resolver el problema de ruteo",
            "details": str(e)
        }), 500

if __name__ == "__main__":
    app.run(debug=True, port=5000)
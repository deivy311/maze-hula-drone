# Drone Challenge 2 - Object Detection in Maze

Este documento describe las modificaciones realizadas al entorno de simulación para cumplir con las reglas del **Drone Challenge 2 - Object Detection in Maze**.

## Características Implementadas

### 1. Sistema de Fases

El entorno ahora soporta dos fases de competencia:

- **Discovery Phase**: Fase de exploración sin objetos (15 minutos)
  - Usar `--phase discovery` al generar el mundo
  - No se colocan objetos en el laberinto
  - Permite explorar y mapear la estructura del laberinto

- **Race Phase**: Fase de carrera con objetos (15 minutos)
  - Usar `--phase race` al generar el mundo
  - Los objetos se colocan a **0.90m de altura** según las reglas
  - Los objetos deben ser detectados e identificados correctamente

### 2. Objetos en el Laberinto

Los objetos son **imágenes pegadas en las paredes** del laberinto, no objetos 3D físicos. Se usan las imágenes reales del modelo ONNX:

- **house**: `house.jpg` - Imagen de una casa
- **tank**: `tank.jpg` - Imagen de un tanque
- **tree**: `tree.jpg` - Imagen de un árbol

Cada objeto:
- Se coloca como una **imagen en la pared** a **0.90m de altura** (según reglas del desafío)
- Tiene una **dirección** (North, East, South, West) que indica en qué pared se coloca:
  - **North**: Pared Norte (arriba) de la celda
  - **East**: Pared Este (derecha) de la celda
  - **South**: Pared Sur (abajo) de la celda
  - **West**: Pared Oeste (izquierda) de la celda
- El tamaño de la imagen es de **0.3m x 0.3m** (30cm)

### 3. Configuración de Objetos

Los objetos se configuran mediante un archivo JSON con el siguiente formato:

```json
[
  {
    "x": 1,
    "y": 4,
    "direction": "North",
    "type": "tank"
  },
  {
    "x": 3,
    "y": 4,
    "direction": "East",
    "type": "house"
  }
]
```

**Campos:**
- `x`, `y`: Coordenadas de la celda (índices, no metros)
- `direction`: Orientación del objeto (`"North"`, `"East"`, `"South"`, `"West"`)
- `type`: Tipo de objeto (`"house"`, `"tank"`, `"tree"`)

### 4. Uso de la Herramienta

#### Generar mundo en fase Discovery (sin objetos):
```bash
python3 gazebo_gen.py --rows 5 --cols 5 --phase discovery
```

#### Generar mundo en fase Race (con objetos):
```bash
python3 gazebo_gen.py --rows 5 --cols 5 --phase race --objects example_objects.json
```

#### Parámetros adicionales:
```bash
python3 gazebo_gen.py \
  --rows 5 \
  --cols 5 \
  --cellSize 1.0 \
  --phase race \
  --objects example_objects.json \
  --seed 42 \
  --out gazebo/worlds/generated_maze.world
```

## Reglas del Desafío Implementadas

✅ **Objetos a 90cm de altura**: Los objetos se colocan a 0.90m de altura

✅ **Orientación de objetos**: Los objetos pueden orientarse hacia North, East, South o West

✅ **Fases separadas**: Discovery Phase (sin objetos) y Race Phase (con objetos)

✅ **Configuración desde GUI**: Los objetos se pueden configurar mediante archivo JSON (que puede ser generado desde la GUI)

## Próximos Pasos

Las siguientes características aún están pendientes de implementación:

1. **Integración del detector ONNX**: Integrar `detect_3_object_12_11.onnx` en la simulación
2. **Sistema de detección y validación**: Capturar snapshots cuando se detectan objetos
3. **Configuración desde parámetros**: Permitir configurar tamaño del laberinto, ubicación inicial, y objetos desde parámetros externos (para integración con GUI)

## Ejemplo Completo

Ver `example_objects.json` para un ejemplo de configuración de objetos basado en el documento del desafío.

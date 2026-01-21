# Cómo Ejecutar la Simulación de Gazebo

## Pasos Rápidos

### 1. Generar el mundo

**Fase Discovery (sin objetos):**
```bash
python3 gazebo_gen.py --rows 5 --cols 5 --phase discovery
```

**Fase Race (con objetos):**
```bash
python3 gazebo_gen.py --rows 5 --cols 5 --phase race --objects example_objects.json
```

### 2. Configurar el path de Gazebo

```bash
source setup_gazebo_path.sh
```

O manualmente:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/gazebo/models
```

### 3. Ejecutar Gazebo

```bash
gz sim gazebo/worlds/generated_maze.world
```

## Usando el Script Automático

El script `run_sim.sh` hace todo automáticamente:

```bash
# Fase discovery
./run_sim.sh --regenerate --phase=discovery

# Fase race con objetos
./run_sim.sh --regenerate --phase=race --objects=example_objects.json

# Con parámetros personalizados
./run_sim.sh --regenerate --rows=6 --cols=6 --phase=race --objects=example_objects.json
```

## Ejemplo Completo

```bash
# 1. Generar mundo en fase discovery
python3 gazebo_gen.py --rows 5 --cols 5 --phase discovery

# 2. Configurar path
source setup_gazebo_path.sh

# 3. Ejecutar Gazebo
gz sim gazebo/worlds/generated_maze.world
```

## Notas Importantes

- **Debes ejecutar `setup_gazebo_path.sh`** antes de lanzar Gazebo, de lo contrario los modelos no se encontrarán
- El mundo generado se guarda en `gazebo/worlds/generated_maze.world`
- Si cambias los parámetros, usa `--regenerate` o elimina el archivo para regenerarlo

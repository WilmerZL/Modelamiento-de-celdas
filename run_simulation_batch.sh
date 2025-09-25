#!/bin/bash

# ============================================================================
# Script de Simulación Secuencial 5G - Ejecución Ordenada
# Una simulación a la vez: 1sparse → 1dense → 3sparse → 3dense → etc.
# ============================================================================

# Configuración base
SCRIPT_NAME="nr_multi_cell_optimized"
BASE_OUTPUT_DIR="./simulation_results"
SIMULATION_TIME=15
NUM_UES=30
EMBB_RATIO=0.6
ISD=200
SCHEDULER="TdmaQos"
HO_ALGORITHM="A2A4"

# Configuraciones de simulación - ORDEN ESPECÍFICO
CELL_NUMBERS=(1 3 5 7 9)
SCENARIOS=("false" "true")  # false=sparse, true=dense
SCENARIO_NAMES=("sparse" "dense")
SEEDS=(1)  # Múltiples semillas para robustez estadística

# Tiempo estimado por simulación (en segundos)
ESTIMATED_TIME_PER_SIM=180

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# ==================== FUNCIONES DE LOGGING ====================
log() {
    echo -e "${BLUE}[$(date '+%H:%M:%S')]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR $(date '+%H:%M:%S')]${NC} $1" >&2
}

success() {
    echo -e "${GREEN}[SUCCESS $(date '+%H:%M:%S')]${NC} $1"
}

warning() {
    echo -e "${YELLOW}[WARNING $(date '+%H:%M:%S')]${NC} $1"
}

info() {
    echo -e "${CYAN}[INFO $(date '+%H:%M:%S')]${NC} $1"
}

progress() {
    echo -e "${MAGENTA}[PROGRESS $(date '+%H:%M:%S')]${NC} $1"
}

# ==================== FUNCIONES DE VALIDACIÓN ====================
check_requirements() {
    log "Verificando requisitos del sistema..."
    
    # Verificar ns3
    if ! command -v ./ns3 &> /dev/null; then
        error "ns3 no encontrado. Asegúrate de estar en el directorio raíz de ns-3."
        return 1
    fi
    
    # Verificar script de simulación
    if [ ! -f "scratch/${SCRIPT_NAME}.cc" ]; then
        error "Script de simulación no encontrado: scratch/${SCRIPT_NAME}.cc"
        return 1
    fi
    
    # Verificar espacio en disco (al menos 1GB)
    available_space=$(df . | awk 'NR==2 {print $4}')
    if [ "$available_space" -lt 1048576 ]; then
        warning "Poco espacio en disco disponible. Se recomiendan al menos 1GB."
    fi
    
    success "Verificación de requisitos completada"
    return 0
}

estimate_total_time() {
    local total_sims=$((${#CELL_NUMBERS[@]} * ${#SCENARIOS[@]} * ${#SEEDS[@]}))
    local estimated_seconds=$((total_sims * ESTIMATED_TIME_PER_SIM))
    local hours=$((estimated_seconds / 3600))
    local minutes=$(((estimated_seconds % 3600) / 60))
    
    info "Tiempo estimado total: ${hours}h ${minutes}m para $total_sims simulaciones"
    info "Tiempo promedio por simulación: $((ESTIMATED_TIME_PER_SIM / 60)) minutos"
}

# ==================== FUNCIÓN PRINCIPAL DE SIMULACIÓN ====================
run_single_simulation() {
    local num_cells=$1
    local dense_flag=$2
    local scenario_name=$3
    local seed=$4
    local sim_number=$5
    local total_sims=$6
    
    echo ""
    echo "╔════════════════════════════════════════════════════════════════════════════════╗"
    progress "SIMULACIÓN $sim_number/$total_sims"
    echo "║ Configuración: $num_cells celdas - Escenario $scenario_name - Semilla $seed"
    echo "║ $(date)"
    echo "╚════════════════════════════════════════════════════════════════════════════════╝"
    
    # Calcular número de UEs según escenario
    local ues_for_scenario=$NUM_UES
    if [ "$dense_flag" == "true" ]; then
        ues_for_scenario=$((NUM_UES * 3 / 2))  # 50% más UEs en escenario denso
    fi
    
    # Directorio de salida específico
    local output_dir="${BASE_OUTPUT_DIR}/${num_cells}cell_${scenario_name}_seed${seed}"
    mkdir -p "$output_dir"
    
    # Archivo de log para esta simulación
    local log_file="${output_dir}/simulation.log"
    
    log "Configuración detallada:"
    echo "   • Número de celdas: $num_cells"
    echo "   • Escenario: $scenario_name ($dense_flag)"
    echo "   • Número de UEs: $ues_for_scenario"
    echo "   • Semilla RNG: $seed"
    echo "   • Directorio salida: $output_dir"
    echo ""
    
    # Comando de simulación
    local cmd="./ns3 run \"scratch/$SCRIPT_NAME.cc \
        --numCells=$num_cells \
        --numUEs=$ues_for_scenario \
        --embbRatio=$EMBB_RATIO \
        --ISD=$ISD \
        --simTime=$SIMULATION_TIME \
        --outputDir=$output_dir \
        --scheduler=$SCHEDULER \
        --hoAlgorithm=$HO_ALGORITHM \
        --denseScenario=$dense_flag \
        --rngSeed=$seed\""
    
    log "Ejecutando simulación..."
    echo "Comando: $cmd" | tee "$log_file"
    echo ""
    
    # Mostrar progreso estimado
    local remaining_sims=$((total_sims - sim_number + 1))
    local remaining_time=$((remaining_sims * ESTIMATED_TIME_PER_SIM))
    local remaining_hours=$((remaining_time / 3600))
    local remaining_minutes=$(((remaining_time % 3600) / 60))
    info "Tiempo restante estimado: ${remaining_hours}h ${remaining_minutes}m"
    
    # Ejecutar simulación
    local start_time=$(date +%s)
    echo "--- INICIO DE SIMULACIÓN $(date) ---" >> "$log_file"
    
    if eval $cmd >> "$log_file" 2>&1; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        local duration_min=$((duration / 60))
        local duration_sec=$((duration % 60))
        
        echo "--- FIN DE SIMULACIÓN $(date) ---" >> "$log_file"
        success "✅ Simulación completada en ${duration_min}m ${duration_sec}s"
        
        # Verificar archivos de salida
        verify_output_files "$output_dir" "$num_cells"
        
        return 0
    else
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        local duration_min=$((duration / 60))
        
        echo "--- ERROR EN SIMULACIÓN $(date) ---" >> "$log_file"
        error "❌ Simulación falló después de ${duration_min} minutos"
        return 1
    fi
}

# ==================== VERIFICACIÓN DE ARCHIVOS DE SALIDA ====================
verify_output_files() {
    local output_dir=$1
    local num_cells=$2
    
    log "Verificando archivos de salida..."
    
    local expected_files=(
        "$output_dir/flow_stats_optimized_${num_cells}cell.csv"
        "$output_dir/cell_stats_optimized_${num_cells}cell.csv"
        "$output_dir/system_stats_optimized_${num_cells}cell.csv"
        "$output_dir/simulation_config_optimized_${num_cells}cell.txt"
    )
    
    local all_files_ok=true
    for file in "${expected_files[@]}"; do
        if [ -f "$file" ] && [ -s "$file" ]; then
            success "   ✓ $(basename $file) - $(du -h "$file" | cut -f1)"
        else
            error "   ✗ $(basename $file) - FALTANTE O VACÍO"
            all_files_ok=false
        fi
    done
    
    if [ "$all_files_ok" = true ]; then
        success "Todos los archivos generados correctamente"
        return 0
    else
        warning "Algunos archivos faltantes"
        return 1
    fi
}

# ==================== FUNCIÓN DE RESUMEN DE PROGRESO ====================
show_progress_summary() {
    local completed=$1
    local total=$2
    local failed=$3
    
    echo ""
    echo "╔════════════════════════════════════════════════════════════════════════════════╗"
    echo "║                               PROGRESO ACTUAL                                 ║"
    echo "╚════════════════════════════════════════════════════════════════════════════════╝"
    echo "   📊 Simulaciones completadas: $completed/$total"
    echo "   ✅ Exitosas: $((completed - failed))"
    echo "   ❌ Fallidas: $failed"
    echo "   📈 Progreso: $(( (completed * 100) / total ))%"
    
    if [ $completed -lt $total ]; then
        local remaining=$((total - completed))
        local remaining_time=$((remaining * ESTIMATED_TIME_PER_SIM))
        local remaining_hours=$((remaining_time / 3600))
        local remaining_minutes=$(((remaining_time % 3600) / 60))
        echo "   ⏱️  Tiempo restante estimado: ${remaining_hours}h ${remaining_minutes}m"
    fi
    echo ""
}

# ==================== GENERAR REPORTE CONSOLIDADO ====================
generate_quick_report() {
    local completed_sims=$1
    local failed_sims=$2
    
    log "Generando reporte rápido..."
    
    local quick_report="$BASE_OUTPUT_DIR/progress_report.txt"
    
    {
        echo "REPORTE DE PROGRESO - $(date)"
        echo "============================================"
        echo "Simulaciones completadas: $completed_sims"
        echo "Simulaciones fallidas: $failed_sims"
        echo "Tasa de éxito: $(( (completed_sims - failed_sims) * 100 / completed_sims ))%"
        echo ""
        echo "DIRECTORIOS GENERADOS:"
        find "$BASE_OUTPUT_DIR" -maxdepth 1 -type d -name "*cell_*" | sort
        echo ""
        echo "ARCHIVOS CSV GENERADOS:"
        find "$BASE_OUTPUT_DIR" -name "*.csv" | wc -l
        echo ""
    } > "$quick_report"
    
    success "Reporte rápido generado: $quick_report"
}

# ==================== FUNCIÓN PRINCIPAL ====================
main() {
    echo ""
    echo "╔════════════════════════════════════════════════════════════════════════════════╗"
    echo "║                    SIMULADOR SECUENCIAL 5G NR - OPTIMIZADO                   ║"
    echo "║                         Ejecución Ordenada Una a Una                         ║"
    echo "╚════════════════════════════════════════════════════════════════════════════════╝"
    echo ""
    
    # Verificar requisitos
    if ! check_requirements; then
        exit 1
    fi
    
    # Crear directorio base
    mkdir -p "$BASE_OUTPUT_DIR"
    
    # Calcular total de simulaciones
    local total_sims=$((${#CELL_NUMBERS[@]} * ${#SCENARIOS[@]} * ${#SEEDS[@]}))
    
    # Mostrar plan de ejecución
    echo "📋 PLAN DE EJECUCIÓN SECUENCIAL:"
    echo "   • Total de simulaciones: $total_sims"
    echo "   • Configuraciones de celdas: ${CELL_NUMBERS[*]}"
    echo "   • Escenarios por configuración: ${SCENARIO_NAMES[*]}"
    echo "   • Semillas por escenario: ${SEEDS[*]}"
    echo "   • Tiempo de simulación: ${SIMULATION_TIME}s cada una"
    echo ""
    echo "🔄 ORDEN DE EJECUCIÓN:"
    
    local sim_counter=1
    for cells in "${CELL_NUMBERS[@]}"; do
        for scenario_idx in "${!SCENARIOS[@]}"; do
            local scenario_name="${SCENARIO_NAMES[$scenario_idx]}"
            for seed in "${SEEDS[@]}"; do
                echo "   $sim_counter. $cells celdas - $scenario_name - semilla $seed"
                sim_counter=$((sim_counter + 1))
            done
        done
    done
    echo ""
    
    estimate_total_time
    
    # Confirmar ejecución
    echo ""
    read -p "¿Continuar con la ejecución secuencial? (y/N): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log "Ejecución cancelada por el usuario"
        exit 0
    fi
    
    # Registrar inicio
    local overall_start_time=$(date +%s)
    local start_date=$(date)
    
    log "🚀 Iniciando ejecución secuencial a las $start_date"
    
    # Variables de seguimiento
    local successful_sims=0
    local failed_sims=0
    local sim_number=1
    
    # ==================== BUCLE PRINCIPAL SECUENCIAL ====================
    for cells in "${CELL_NUMBERS[@]}"; do
        for scenario_idx in "${!SCENARIOS[@]}"; do
            local dense="${SCENARIOS[$scenario_idx]}"
            local scenario_name="${SCENARIO_NAMES[$scenario_idx]}"
            
            for seed in "${SEEDS[@]}"; do
                
                # Ejecutar simulación individual
                if run_single_simulation "$cells" "$dense" "$scenario_name" "$seed" "$sim_number" "$total_sims"; then
                    successful_sims=$((successful_sims + 1))
                    success "Simulación $sim_number completada exitosamente"
                else
                    failed_sims=$((failed_sims + 1))
                    error "Simulación $sim_number falló"
                fi
                
                # Mostrar resumen de progreso
                show_progress_summary $sim_number $total_sims $failed_sims
                
                # Generar reporte intermedio cada 5 simulaciones
                if [ $((sim_number % 5)) -eq 0 ] || [ $sim_number -eq $total_sims ]; then
                    generate_quick_report $sim_number $failed_sims
                fi
                
                sim_number=$((sim_number + 1))
                
                # Pausa entre simulaciones (excepto la última)
                if [ $sim_number -le $total_sims ]; then
                    log "Pausa de 5 segundos antes de la siguiente simulación..."
                    sleep 5
                fi
                
            done
        done
    done
    
    # ==================== RESUMEN FINAL ====================
    local overall_end_time=$(date +%s)
    local total_time=$((overall_end_time - overall_start_time))
    local hours=$((total_time / 3600))
    local minutes=$(((total_time % 3600) / 60))
    local seconds=$((total_time % 60))
    
    echo ""
    echo "╔════════════════════════════════════════════════════════════════════════════════╗"
    echo "║                              ✅ EJECUCIÓN COMPLETADA                          ║"
    echo "╚════════════════════════════════════════════════════════════════════════════════╝"
    echo ""
    
    success "🎉 Todas las simulaciones procesadas"
    echo ""
    echo "📊 ESTADÍSTICAS FINALES:"
    echo "   • Total ejecutadas: $total_sims"
    echo "   • Exitosas: $successful_sims"
    echo "   • Fallidas: $failed_sims"
    echo "   • Tasa de éxito: $(( successful_sims * 100 / total_sims ))%"
    echo "   • Tiempo total: ${hours}h ${minutes}m ${seconds}s"
    echo "   • Promedio por simulación: $((total_time / total_sims))s"
    echo ""
    
    echo "📁 ESTRUCTURA DE RESULTADOS GENERADA:"
    echo "   $BASE_OUTPUT_DIR/"
    find "$BASE_OUTPUT_DIR" -maxdepth 1 -type d -name "*cell_*" | sort | head -10 | while read dir; do
        echo "   ├── $(basename "$dir")/"
    done
    if [ $(find "$BASE_OUTPUT_DIR" -maxdepth 1 -type d -name "*cell_*" | wc -l) -gt 10 ]; then
        echo "   └── ... (y más)"
    fi
    echo ""
    
    echo "📈 ARCHIVOS GENERADOS:"
    local csv_count=$(find "$BASE_OUTPUT_DIR" -name "*.csv" | wc -l)
    local txt_count=$(find "$BASE_OUTPUT_DIR" -name "*.txt" | wc -l)
    echo "   • Archivos CSV: $csv_count"
    echo "   • Archivos de configuración: $txt_count"
    echo "   • Logs de simulación: $(find "$BASE_OUTPUT_DIR" -name "simulation.log" | wc -l)"
    echo ""
    
    echo "🔬 PRÓXIMOS PASOS PARA ANÁLISIS:"
    echo "   1. Todos los resultados están listos para análisis"
    echo "   2. Cada directorio contiene:"
    echo "      • flow_stats_optimized_*cell.csv (métricas por flujo)"
    echo "      • cell_stats_optimized_*cell.csv (métricas por celda)" 
    echo "      • system_stats_optimized_*cell.csv (métricas del sistema)"
    echo "      • simulation_config_optimized_*cell.txt (configuración)"
    echo ""
    echo "   3. Para análisis consolidado, puedes crear scripts que procesen"
    echo "      todos los archivos CSV generados"
    echo ""
    
    if [ $failed_sims -eq 0 ]; then
        success "🏆 ¡PERFECTO! Todas las simulaciones completadas exitosamente"
    else
        warning "⚠️  $failed_sims simulaciones fallaron - revisar logs individuales"
        echo ""
        echo "📋 Para revisar fallos:"
        echo "   grep -r \"ERROR\" $BASE_OUTPUT_DIR/*/simulation.log"
    fi
    
    echo ""
    echo "╚═══════════════════════════════════════════════════════════════════════════════╝"
    
    # Crear script simple de consolidación
    cat > "$BASE_OUTPUT_DIR/consolidate_results.py" << 'EOF'
#!/usr/bin/env python3
"""
Script básico para consolidar resultados de simulaciones 5G
"""
import pandas as pd
import os
import glob

def consolidate_system_stats(base_dir):
    """Consolidar todas las estadísticas del sistema"""
    
    pattern = os.path.join(base_dir, "*/system_stats_optimized_*cell.csv")
    files = glob.glob(pattern)
    
    all_data = []
    
    for file in files:
        # Extraer info del path
        dir_name = os.path.basename(os.path.dirname(file))
        parts = dir_name.split('_')
        
        if len(parts) >= 3:
            cells = parts[0].replace('cell', '')
            scenario = parts[1]
            seed = parts[2].replace('seed', '')
            
            # Leer archivo
            try:
                df = pd.read_csv(file)
                df['NumCells'] = cells
                df['Scenario'] = scenario  
                df['Seed'] = seed
                df['SourceFile'] = file
                all_data.append(df)
            except Exception as e:
                print(f"Error leyendo {file}: {e}")
    
    if all_data:
        consolidated = pd.concat(all_data, ignore_index=True)
        output_file = os.path.join(base_dir, "consolidated_system_stats.csv")
        consolidated.to_csv(output_file, index=False)
        print(f"✓ Datos consolidados guardados en: {output_file}")
        print(f"  Total de archivos procesados: {len(files)}")
        print(f"  Total de registros: {len(consolidated)}")
        return output_file
    else:
        print("✗ No se encontraron archivos para consolidar")
        return None

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Uso: python3 consolidate_results.py <directorio_base>")
        sys.exit(1)
    
    base_dir = sys.argv[1]
    consolidate_system_stats(base_dir)
EOF
    
    chmod +x "$BASE_OUTPUT_DIR/consolidate_results.py"
    
    echo ""
    success "Script de consolidación creado: $BASE_OUTPUT_DIR/consolidate_results.py"
    echo "Para consolidar resultados: python3 $BASE_OUTPUT_DIR/consolidate_results.py $BASE_OUTPUT_DIR"
    echo ""
}

# Ejecutar función principal
main "$@"

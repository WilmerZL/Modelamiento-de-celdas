# Modelamiento de Celdas 5G con NS-3

Este repositorio contiene una simulación optimizada de redes 5G NR multi-celda utilizando NS-3.43 con el módulo 5G-LENA. La simulación modela escenarios realistas de comunicación eMBB (enhanced Mobile Broadband) y URLLC (Ultra-Reliable Low Latency Communications) en entornos urbanos densos y suburbanos dispersos.

## Características Principales

### Arquitectura de Red
- **Tecnología**: 5G NR (New Radio) FR1 en banda de 3.5 GHz
- **Configuraciones**: 1, 3, 5, 7, o 9 celdas con distribución hexagonal optimizada
- **Ancho de banda**: 100 MHz con numerología 2 (30 kHz SCS)
- **Beamforming**: Algoritmos realistas con retroalimentación MIMO/CSI

### Tipos de Tráfico Soportados
- **eMBB**: Video streaming con tasas adaptativas (5-20 Mbps por UE)
- **URLLC**: Control crítico con latencia ultra-baja (< 5 ms)
- **QoS**: Scheduling diferenciado con TdmaQos/OfdmaQos

### Escenarios de Simulación
- **Denso Urbano (UMa)**: Alta densidad de usuarios, propagación compleja
- **Disperso Suburbano (RMa)**: Cobertura extendida, menos interferencia
- **Handover**: Algoritmos A2A4 con umbrales optimizados
## Parámetros de Configuración

| Parámetro | Descripción | Valores | Por Defecto |
|-----------|-------------|---------|-------------|
| `numCells` | Número de celdas | 1,3,5,7,9 | 1 |
| `numUEs` | Total de UEs | 10-100 | 30 |
| `embbRatio` | Proporción eMBB vs URLLC | 0.0-1.0 | 0.6 |
| `ISD` | Distancia inter-sitio (m) | 100-1000 | 200 |
| `simTime` | Tiempo simulación (s) | 5-60 | 15 |
| `denseScenario` | Escenario denso | true/false | false |
| `scheduler` | Algoritmo scheduling | TdmaQos/OfdmaQos | TdmaQos |
| `hoAlgorithm` | Algoritmo handover | A2A4/A3 | A2A4 |
| `rngSeed` | Semilla aleatoria | 1-999 | 1 |


## Optimizaciones Implementadas

### Mejoras de Rendimiento
- **Numerología 2**: SCS de 30 kHz para menor latencia
- **Potencia UE optimizada**: 26 dBm para mejor SINR
- **Distribución espacial**: Layouts hexagonales optimizados

### Modelado Realista
- **Propagación 3GPP**: UMa urbano / RMa rural
- **Beamforming avanzado**: Con retroalimentación CSI
- **Tráfico heterogéneo**: eMBB y URLLC diferenciados
## Referencias

- 3GPP TS 38.series - 5G NR Specifications
- NS-3 Documentation - https://www.nsnam.org/
- 5G-LENA Module - https://5g-lena.cttc.es/


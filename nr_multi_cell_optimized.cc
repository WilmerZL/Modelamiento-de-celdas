// ============================================================================
//   Modelamiento de celdas 5G - FR1 3.5 GHz
// ============================================================================

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>
#include <filesystem>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OptimizedMultiCellNr");

// ================== Estructuras para métricas avanzadas ===================
struct ChannelMetrics {
    double sumSinrDb = 0.0;
    double sumRsrpDbm = 0.0;
    double sumRsrqDb = 0.0;
    uint32_t samples = 0;
    double maxSinr = -1000.0;
    double minSinr = 1000.0;
};

struct QoEMetrics {
    double totalDelay = 0.0;
    double totalJitter = 0.0;
    uint64_t totalPackets = 0;
    double sumThroughput = 0.0;
    uint32_t flows = 0;
};

// Variables globales para métricas
static std::unordered_map<uint64_t, ChannelMetrics> g_channelMetrics;
static std::unordered_map<uint32_t, QoEMetrics> g_cellQoE;
static std::unordered_map<uint64_t, uint32_t> g_imsiToCell;
static std::unordered_map<uint64_t, double> g_imsiDistance;
static std::unordered_map<uint32_t, uint32_t> g_cellUeCount;
static std::unordered_map<uint64_t, std::vector<double>> g_sinrHistory;


// Contadores de handover
static uint32_t g_handoverAttempts = 0;
static uint32_t g_handoverSuccess = 0;
static uint32_t g_handoverFailures = 0;

// ==================== Callbacks para métricas ============================
static void
EnhancedSinrCallback(uint64_t imsi, RxPacketTraceParams params)
{
    if (params.m_sinr > 0.0) {
        double sinrDb = 10.0 * std::log10(params.m_sinr);
        
        auto& metrics = g_channelMetrics[imsi];
        metrics.sumSinrDb += sinrDb;
        metrics.samples++;
        metrics.maxSinr = std::max(metrics.maxSinr, sinrDb);
        metrics.minSinr = std::min(metrics.minSinr, sinrDb);
        
        // Historial para análisis de variabilidad
        g_sinrHistory[imsi].push_back(sinrDb);
        if (g_sinrHistory[imsi].size() > 1000) {
            g_sinrHistory[imsi].erase(g_sinrHistory[imsi].begin());
        }
    }
}

static void
RsrpCallback(uint64_t imsi, uint16_t cellId, double rsrp)
{
    g_channelMetrics[imsi].sumRsrpDbm += rsrp;
}

static void
RsrqCallback(uint64_t imsi, uint16_t cellId, double rsrq)
{
    g_channelMetrics[imsi].sumRsrqDb += rsrq;
}

static void
HandoverStartCallback(uint64_t imsi, uint16_t sourceCellId, uint16_t targetCellId)
{
    g_handoverAttempts++;
}

static void
HandoverSuccessCallback(uint64_t imsi, uint16_t sourceCellId, uint16_t targetCellId)
{
    g_handoverSuccess++;
}

static void
HandoverFailureCallback(uint64_t imsi, uint16_t sourceCellId, uint16_t targetCellId)
{
    g_handoverFailures++;
}

// ==================== Funciones de distribución espacial ==================
enum ScenarioType {
    DENSE_URBAN = 0,
    SPARSE_SUBURBAN = 1
};

static Ptr<ListPositionAllocator>
CreateOptimizedCellLayout(uint32_t numCells, double ISD, double baseHeight, ScenarioType scenario)
{
    auto positions = CreateObject<ListPositionAllocator>();
    
    // Ajustar ISD según el escenario
    double effectiveISD = (scenario == DENSE_URBAN) ? ISD * 0.7 : ISD * 1.3;
    
    switch (numCells) {
        case 1:
            positions->Add(Vector(0.0, 0.0, baseHeight));
            break;
            
        case 3: {
            // Triángulo equilátero optimizado
            double r = effectiveISD * 0.577; // radio del circuncentro
            positions->Add(Vector(0.0, r, baseHeight));
            positions->Add(Vector(-r * 0.866, -r * 0.5, baseHeight));
            positions->Add(Vector(r * 0.866, -r * 0.5, baseHeight));
            break;
        }
        
        case 5: {
            // Centro + cruz optimizada
            positions->Add(Vector(0.0, 0.0, baseHeight));
            double offset = effectiveISD * 0.7;
            positions->Add(Vector(offset, 0.0, baseHeight));
            positions->Add(Vector(-offset, 0.0, baseHeight));
            positions->Add(Vector(0.0, offset, baseHeight));
            positions->Add(Vector(0.0, -offset, baseHeight));
            break;
        }
        
        case 7: {
            // Hexágono con centro
            positions->Add(Vector(0.0, 0.0, baseHeight));
            double r = effectiveISD * 0.6;
            for (int i = 0; i < 6; i++) {
                double angle = i * M_PI / 3.0;
                positions->Add(Vector(r * cos(angle), r * sin(angle), baseHeight));
            }
            break;
        }
        
        case 9:
        default: {
            // Centro + 8 direcciones
            positions->Add(Vector(0.0, 0.0, baseHeight));
            double r = effectiveISD * 0.65;
            for (int i = 0; i < 8; i++) {
                double angle = i * M_PI / 4.0;
                positions->Add(Vector(r * cos(angle), r * sin(angle), baseHeight));
            }
            break;
        }
    }
    
    return positions;
}

static void
DistributeUsersOptimized(NodeContainer ueNodes, NodeContainer gnbNodes, 
                        ScenarioType scenario, double ISD, double userHeight)
{
    Ptr<UniformRandomVariable> uniformRv = CreateObject<UniformRandomVariable>();
    Ptr<ExponentialRandomVariable> expRv = CreateObject<ExponentialRandomVariable>();
    
    uint32_t numUEs = ueNodes.GetN();
    uint32_t numCells = gnbNodes.GetN();
    
    // Distribución por celda con variabilidad realista
    std::vector<uint32_t> uesPerCell(numCells);
    uint32_t baseUesPerCell = numUEs / numCells;
    uint32_t remainder = numUEs % numCells;
    
    for (uint32_t i = 0; i < numCells; i++) {
        uesPerCell[i] = baseUesPerCell + (i < remainder ? 1 : 0);
        
        // Variabilidad según escenario
        if (scenario == DENSE_URBAN) {
            // Más concentración en algunas celdas
            if (i == 0 || i == numCells/2) {
                uesPerCell[i] *= 1.5;
            }
        }
    }
    
    uint32_t ueIndex = 0;
    for (uint32_t cellId = 0; cellId < numCells && ueIndex < numUEs; cellId++) {
        Vector cellPos = gnbNodes.Get(cellId)->GetObject<MobilityModel>()->GetPosition();
        
        // Radio de cobertura según escenario
        double maxRadius = (scenario == DENSE_URBAN) ? ISD * 0.4 : ISD * 0.8;
        double minRadius = (scenario == DENSE_URBAN) ? 10.0 : 50.0;
        
        for (uint32_t j = 0; j < uesPerCell[cellId] && ueIndex < numUEs; j++) {
            double radius, angle;
            
            if (scenario == DENSE_URBAN) {
                // Distribución más concentrada cerca del centro
                radius = minRadius + expRv->GetValue() * (maxRadius - minRadius) * 0.3;
                radius = std::min(radius, maxRadius);
            } else {
                // Distribución más uniforme
                radius = uniformRv->GetValue(minRadius, maxRadius);
            }
            
            angle = uniformRv->GetValue(0.0, 2 * M_PI);
            
            double x = cellPos.x + radius * cos(angle);
            double y = cellPos.y + radius * sin(angle);
            
            ueNodes.Get(ueIndex)->GetObject<MobilityModel>()->SetPosition(
                Vector(x, y, userHeight));
            
            ueIndex++;
        }
    }
    
    // Distribuir UEs restantes aleatoriamente
    double areaSize = ISD * 1.5;
    for (; ueIndex < numUEs; ueIndex++) {
        double x = uniformRv->GetValue(-areaSize, areaSize);
        double y = uniformRv->GetValue(-areaSize, areaSize);
        ueNodes.Get(ueIndex)->GetObject<MobilityModel>()->SetPosition(
            Vector(x, y, userHeight));
    }
}

// ==================== Función principal ====================================
int main(int argc, char** argv)
{
    // Parámetros configurables - EXACTOS como tu código
    uint32_t numCells = 1;
    uint32_t numUEs = 30;
    double embbRatio = 0.6;
    double ISD = 200.0;
    double simTime = 15.0;
    double appStartTime = 5.0;
    uint32_t rngSeed = 1;
    std::string outputDir = "./results";
    std::string scheduler = "TdmaQos";
    std::string hoAlgorithm = "A2A4";
    bool denseScenario = false;
    
    // ==================== CAMBIO 1: Solo potencia UE añadida ====================
    // Parámetros del canal - mantengo gnbTxPower igual, solo añado ueTxPower
    double gnbTxPower = 46.0; // IGUAL que tu código
    double ueTxPower = 26.0;  // CAMBIO 2: Solo añadir esto
    double gnbHeight = 25.0;  // IGUAL que tu código
    double ueHeight = 1.5;    // IGUAL que tu código
    
    CommandLine cmd(__FILE__);
    cmd.AddValue("numCells", "Número de celdas (1,3,5,7,9)", numCells);
    cmd.AddValue("numUEs", "Número total de UEs", numUEs);
    cmd.AddValue("embbRatio", "Proporción de UEs eMBB", embbRatio);
    cmd.AddValue("ISD", "Distancia inter-sitio (m)", ISD);
    cmd.AddValue("simTime", "Tiempo de simulación (s)", simTime);
    cmd.AddValue("rngSeed", "Semilla aleatoria", rngSeed);
    cmd.AddValue("outputDir", "Directorio de salida", outputDir);
    cmd.AddValue("scheduler", "Scheduler (TdmaQos|OfdmaQos)", scheduler);
    cmd.AddValue("hoAlgorithm", "Algoritmo de handover", hoAlgorithm);
    cmd.AddValue("denseScenario", "Escenario denso (true) o disperso (false)", denseScenario);
    cmd.Parse(argc, argv);
    
    // Configurar directorios de salida - IGUAL que tu código
    std::string scenarioName = denseScenario ? "dense" : "sparse";
    std::ostringstream dirStream;
    dirStream << outputDir; 
    outputDir = dirStream.str();
    std::filesystem::create_directories(outputDir);
    
    // Inicialización - IGUAL que tu código
    SeedManager::SetSeed(rngSeed);
    
    // Crear nodos - IGUAL que tu código
    NodeContainer gnbNodes, ueNodes;
    gnbNodes.Create(numCells);
    ueNodes.Create(numUEs);
    
    // Configurar movilidad de gNBs - IGUAL que tu código
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    ScenarioType scenario = denseScenario ? DENSE_URBAN : SPARSE_SUBURBAN;
    Ptr<ListPositionAllocator> gnbPositions = CreateOptimizedCellLayout(
        numCells, ISD, gnbHeight, scenario);
    gnbMobility.SetPositionAllocator(gnbPositions);
    gnbMobility.Install(gnbNodes);
    
    // Configurar movilidad de UEs - IGUAL que tu código
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);
    DistributeUsersOptimized(ueNodes, gnbNodes, scenario, ISD, ueHeight);
    
    // Configurar NR Helper con beamforming mejorado - IGUAL que tu código
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    
    Ptr<RealisticBeamformingHelper> beamformingHelper = CreateObject<RealisticBeamformingHelper>();
    beamformingHelper->SetBeamformingMethod(RealisticBeamformingAlgorithm::GetTypeId());
    
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetGnbBeamManagerTypeId(RealisticBfManager::GetTypeId());
    
    // Configuraciones avanzadas - IGUAL que tu código
    nrHelper->SetAttribute("EnableMimoFeedback", BooleanValue(true));
    nrHelper->SetAttribute("CsiFeedbackFlags", UintegerValue(7)); // Todas las banderas CSI
    
    // Scheduler optimizado - IGUAL que tu código
    std::string schedulerTypeId = "ns3::NrMacScheduler" + scheduler;
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(schedulerTypeId));
    nrHelper->SetSchedulerAttribute("SchedLcAlgorithmType", 
        TypeIdValue(NrMacSchedulerLcQos::GetTypeId()));
    
    // ==================== CAMBIO 7: Solo handover optimizado ====================
    // Configurar handover - CAMBIO: parámetros optimizados
    if (hoAlgorithm == "A2A4") {
        nrHelper->SetHandoverAlgorithmType("ns3::A2A4RsrqHandoverAlgorithm");
        nrHelper->SetHandoverAlgorithmAttribute("ServingCellThreshold", UintegerValue(15)); // CAMBIO: 15 vs 18
        nrHelper->SetHandoverAlgorithmAttribute("NeighbourCellOffset", UintegerValue(3)); // CAMBIO: 3 vs 5
    }
    
    // Configurar banda y canal - IGUAL que tu código base
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 100e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    
    // ==================== CAMBIO 5: Solo modelo propagación optimizado ====================
    // Modelo de propagación según escenario - CAMBIO: UMa para denso
    std::string propagationModel = denseScenario ? "UMa" : "RMa"; 
    channelHelper->ConfigureFactories(propagationModel, "Default", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
    
    // Instalar dispositivos - IGUAL que tu código
    NetDeviceContainer gnbDevices = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
    
    // ==================== CAMBIO 1: Solo numerología optimizada ====================
    // Configurar parámetros de gNB - CAMBIO: numerología 2 vs 1
    for (uint32_t i = 0; i < gnbDevices.GetN(); ++i) {
        Ptr<NrGnbPhy> gnbPhy = nrHelper->GetGnbPhy(gnbDevices.Get(i), 0);
        gnbPhy->SetAttribute("TxPower", DoubleValue(gnbTxPower));
        gnbPhy->SetAttribute("Numerology", UintegerValue(2)); // CAMBIO: 2 vs 1 (30 kHz vs 15 kHz)
    }
    
    // ==================== CAMBIO 2: Solo potencia UE añadida ====================
    // Añadir configuración de potencia UE (no estaba en tu código original)
    for (uint32_t i = 0; i < ueDevices.GetN(); ++i) {
        Ptr<NrUePhy> uePhy = nrHelper->GetUePhy(ueDevices.Get(i), 0);
        uePhy->SetAttribute("TxPower", DoubleValue(ueTxPower)); // CAMBIO: Añadir potencia UE
    }
    
    // Configurar EPC - IGUAL que tu código
    auto [remoteHost, remoteAddr] = epcHelper->SetupRemoteHost("100Gb/s", 1000, Seconds(0));
    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(remoteHost);
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueDevices);
    
    // Clasificar UEs - IGUAL que tu código
    uint32_t numEmbbUEs = static_cast<uint32_t>(embbRatio * numUEs);
    NodeContainer embbUEs, urllcUEs;
    NetDeviceContainer embbDevices, urllcDevices;
    
    for (uint32_t i = 0; i < numUEs; ++i) {
        if (i < numEmbbUEs) {
            embbUEs.Add(ueNodes.Get(i));
            embbDevices.Add(ueDevices.Get(i));
        } else {
            urllcUEs.Add(ueNodes.Get(i));
            urllcDevices.Add(ueDevices.Get(i));
        }
    }
    double embbBudgetBps = denseScenario ? 3e8 /*300 Mb/s*/ : 2e8 /*200 Mb/s*/;
    uint32_t nEmbb = embbUEs.GetN();
    uint64_t perUeRateBps = 0;

    if (nEmbb > 0) {
        double fairShare = embbBudgetBps / static_cast<double>(nEmbb);
        // Piso 5 Mb/s, techo 20 Mb/s por UE para evitar colas y pérdida
        fairShare = std::max(5e6, std::min(fairShare, 20e6));
        perUeRateBps = static_cast<uint64_t>(fairShare);
    } else {
        perUeRateBps = static_cast<uint64_t>(10e6);
    }
    // Configurar aplicaciones - IGUAL que tu código
    uint16_t embbPort = 7000;
    uint16_t urllcPort = 7001;
    ApplicationContainer serverApps, clientApps;
    
    // Aplicaciones eMBB - Video streaming - IGUAL que tu código
    for (uint32_t i = 0; i < embbUEs.GetN(); ++i) {
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", 
            InetSocketAddress(Ipv4Address::GetAny(), embbPort));
        serverApps.Add(sinkHelper.Install(embbUEs.Get(i)));
        
        Ipv4Address destAddr = ueIpIfaces.GetAddress(i);
        OnOffHelper onOffHelper("ns3::UdpSocketFactory", 
            InetSocketAddress(destAddr, embbPort));
        
        // Tráfico variable según escenario - IGUAL que tu código
        onOffHelper.SetAttribute("PacketSize", UintegerValue(1400));
        onOffHelper.SetAttribute("DataRate", DataRateValue(DataRate(perUeRateBps)));
        onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        
        clientApps.Add(onOffHelper.Install(remoteHost));
        
        // Bearer eMBB - IGUAL que tu código
        NrEpsBearer bearer(NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT);
        nrHelper->ActivateDedicatedEpsBearer(embbDevices.Get(i), bearer, Create<NrEpcTft>());
    }
    
    // ==================== CAMBIO 4: Solo intervalo URLLC optimizado ====================
    // Aplicaciones URLLC - Control crítico - CAMBIO: intervalo más frecuente
    for (uint32_t i = 0; i < urllcUEs.GetN(); ++i) {
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", 
            InetSocketAddress(Ipv4Address::GetAny(), urllcPort));
        serverApps.Add(sinkHelper.Install(urllcUEs.Get(i)));
        
        uint32_t ueIdx = i + numEmbbUEs;
        Ipv4Address destAddr = ueIpIfaces.GetAddress(ueIdx);
        UdpClientHelper udpClient(destAddr, urllcPort);
        
        // Configuración URLLC optimizada - CAMBIO: intervalo más frecuente
        uint32_t pktSize = 100; // IGUAL que tu código
        double interval = denseScenario ? 0.0005 : 0.001; // CAMBIO: más frecuente (era 0.001 : 0.002)
        
        udpClient.SetAttribute("PacketSize", UintegerValue(pktSize));
        udpClient.SetAttribute("Interval", TimeValue(Seconds(interval)));
        udpClient.SetAttribute("MaxPackets", UintegerValue(0)); // Ilimitado
        
        clientApps.Add(udpClient.Install(remoteHost));
        
        // Bearer URLLC - IGUAL que tu código
        NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(urllcDevices.Get(i), bearer, Create<NrEpcTft>());
    }
    
    // Conectar UEs a la celda más cercana - IGUAL que tu código
    nrHelper->AttachToClosestGnb(ueDevices, gnbDevices);
    
    // Configurar trazas mejoradas - IGUAL que tu código
    for (uint32_t i = 0; i < ueDevices.GetN(); ++i) {
        Ptr<NrUeNetDevice> ueDevice = ueDevices.Get(i)->GetObject<NrUeNetDevice>();
        uint64_t imsi = ueDevice->GetImsi();
        
        // SINR tracing
        Ptr<NrSpectrumPhy> spectrumPhy = ueDevice->GetPhy(0)->GetSpectrumPhy();
        spectrumPhy->TraceConnectWithoutContext("RxPacketTraceUe", 
            MakeBoundCallback(&EnhancedSinrCallback, imsi));
        
        // Handover tracing
        ueDevice->GetPhy(0)->TraceConnectWithoutContext("HandoverStart", 
            MakeBoundCallback(&HandoverStartCallback, imsi));
        ueDevice->GetPhy(0)->TraceConnectWithoutContext("HandoverSuccess", 
            MakeBoundCallback(&HandoverSuccessCallback, imsi));
        ueDevice->GetPhy(0)->TraceConnectWithoutContext("HandoverFailure", 
            MakeBoundCallback(&HandoverFailureCallback, imsi));
    }
    
    // Calcular asociaciones UE-celda y distancias - IGUAL que tu código
    for (uint32_t i = 0; i < numUEs; ++i) {
        Vector uePos = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        double minDistance = std::numeric_limits<double>::max();
        uint32_t closestCell = 0;
        
        for (uint32_t j = 0; j < numCells; ++j) {
            Vector gnbPos = gnbNodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
            double distance = CalculateDistance(uePos, gnbPos);
            
            if (distance < minDistance) {
                minDistance = distance;
                closestCell = j;
            }
        }
        
        uint64_t imsi = ueDevices.Get(i)->GetObject<NrUeNetDevice>()->GetImsi();
        g_imsiToCell[imsi] = closestCell;
        g_imsiDistance[imsi] = minDistance;
        g_cellUeCount[closestCell]++;
    }
    
    // Configurar y ejecutar simulación - IGUAL que tu código
    Ptr<UniformRandomVariable> appJitter = CreateObject<UniformRandomVariable>();

    for (uint32_t i = 0; i < serverApps.GetN(); ++i) {
        double s = appStartTime + appJitter->GetValue(0.0, 0.5);
        serverApps.Get(i)->SetStartTime(Seconds(s));
        serverApps.Get(i)->SetStopTime(Seconds(simTime));
    }
    for (uint32_t i = 0; i < clientApps.GetN(); ++i) {
        double s = appStartTime + appJitter->GetValue(0.0, 0.5);
        clientApps.Get(i)->SetStartTime(Seconds(s));
        clientApps.Get(i)->SetStopTime(Seconds(simTime));
    }

    FlowMonitorHelper flowMonitorHelper;
    Ptr<FlowMonitor> monitor = flowMonitorHelper.InstallAll();
    
    std::cout << "\n========== SIMULACIÓN CON OPTIMIZACIONES MÍNIMAS ==========\n";
    std::cout << "CAMBIOS APLICADOS (solo los compatibles):\n";
    std::cout << "1. Numerología: 2 (30 kHz) vs 1 (15 kHz original)\n";
    std::cout << "2. Potencia UE: " << ueTxPower << " dBm (nueva)\n";
    std::cout << "4. URLLC intervalo: más frecuente para numerología 2\n";
    std::cout << "5. Propagación: " << propagationModel << " (optimizado)\n";
    std::cout << "7. Handover: umbrales optimizados (15 vs 18, 3 vs 5)\n";
    std::cout << "===========================================================\n\n";
    
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    
    // ==================== Procesamiento de resultados ======================
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = 
        DynamicCast<Ipv4FlowClassifier>(flowMonitorHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    
    // Archivo de estadísticas de flujos - MEJORADO con columna adicional
    std::string flowFile = outputDir + "/flow_stats_optimized_" + std::to_string(numCells) + 
                      "cell.csv";
    std::ofstream flowOut(flowFile);
    flowOut << "FlowId,TrafficType,UeImsi,ServingCell,Distance(m),DstAddr,"
            << "AvgSinr(dB),MinSinr(dB),MaxSinr(dB),SinrStdDev(dB),"
            << "TxPackets,RxPackets,LostPackets,PacketLossRatio(%),"
            << "Throughput(Mbps),MeanDelay(ms),MeanJitter(ms),"
            << "QoEScore,ReliabilityScore,Numerology\n";
    
    struct CellSummary {
        double totalThroughput = 0.0;
        uint64_t totalTx = 0, totalRx = 0, totalLost = 0;
        double totalSinr = 0.0;
        uint32_t sinrSamples = 0;
        QoEMetrics qoe;
    };
    
    std::map<uint32_t, CellSummary> cellSummaries;
    double totalSystemThroughput = 0.0;
    double totalUrllcDelay = 0.0;
    double totalEmbbDelay = 0.0;
    uint32_t urllcFlows = 0;
    uint32_t embbFlows = 0;
    
    for (const auto& flowStat : stats) {
        Ipv4FlowClassifier::FiveTuple flowTuple = classifier->FindFlow(flowStat.first);
        
        // Identificar tipo de tráfico
        bool isEmbb = (flowTuple.destinationPort == embbPort);
        bool isUrllc = (flowTuple.destinationPort == urllcPort);
        if (!isEmbb && !isUrllc) continue;
        
        // Encontrar IMSI del UE
        uint64_t imsi = 0;
        for (uint32_t i = 0; i < numUEs; ++i) {
            if (ueIpIfaces.GetAddress(i) == flowTuple.destinationAddress) {
                imsi = ueDevices.Get(i)->GetObject<NrUeNetDevice>()->GetImsi();
                break;
            }
        }
        
        if (imsi == 0) continue;
        
        // Obtener métricas del canal
        ChannelMetrics& chanMetrics = g_channelMetrics[imsi];
        double avgSinr = (chanMetrics.samples > 0) ? 
                        chanMetrics.sumSinrDb / chanMetrics.samples : 0.0;
        
        // Calcular desviación estándar del SINR
        double sinrStdDev = 0.0;
        if (g_sinrHistory[imsi].size() > 1) {
            double mean = avgSinr;
            double variance = 0.0;
            for (double sinr : g_sinrHistory[imsi]) {
                variance += (sinr - mean) * (sinr - mean);
            }
            variance /= g_sinrHistory[imsi].size() - 1;
            sinrStdDev = std::sqrt(variance);
        }
        
        // Calcular métricas de QoS
        const FlowMonitor::FlowStats& fs = flowStat.second;
        uint64_t lostPackets = fs.txPackets - fs.rxPackets;
        double packetLossRatio = (fs.txPackets > 0) ? 
                                (100.0 * lostPackets / fs.txPackets) : 0.0;
        
        double throughput = 0.0;
        double meanDelay = 0.0;
        double meanJitter = 0.0;
        
        if (fs.rxPackets > 0) {
            double flowDuration = (fs.timeLastRxPacket - fs.timeFirstTxPacket).GetSeconds();
            if (flowDuration > 0) {
                throughput = (fs.rxBytes * 8.0) / (flowDuration * 1e6); // Mbps
            }
            meanDelay = (fs.delaySum.GetSeconds() / fs.rxPackets) * 1000.0; // ms
            
            if (fs.rxPackets > 1) {
                meanJitter = (fs.jitterSum.GetSeconds() / (fs.rxPackets - 1)) * 1000.0; // ms
            }
        }
        
        // Recopilar estadísticas de latencia por tipo
        if (isUrllc) {
            totalUrllcDelay += meanDelay;
            urllcFlows++;
        } else {
            totalEmbbDelay += meanDelay;
            embbFlows++;
        }
        
        // Calcular QoE Score (0-100)
        double qoeScore = 100.0;
        if (isEmbb) {
            // Para eMBB: throughput y delay son críticos
            if (throughput < 25.0) qoeScore *= (throughput / 25.0);
            if (meanDelay > 20.0) qoeScore *= (20.0 / meanDelay);
            if (packetLossRatio > 1.0) qoeScore *= (1.0 / packetLossRatio);
        } else {
            // Para URLLC: latencia ultra-baja es crítica
            if (meanDelay > 5.0) qoeScore *= (5.0 / meanDelay);
            if (packetLossRatio > 0.1) qoeScore *= (0.1 / packetLossRatio);
            if (meanJitter > 2.0) qoeScore *= (2.0 / meanJitter);
        }
        qoeScore = std::max(0.0, std::min(100.0, qoeScore));
        
        // Calcular Reliability Score basado en consistencia del SINR
        double reliabilityScore = 100.0;
        if (chanMetrics.samples > 0) {
            double sinrRange = chanMetrics.maxSinr - chanMetrics.minSinr;
            if (sinrRange > 20.0) { // Penalizar alta variabilidad
                reliabilityScore *= (20.0 / sinrRange);
            }
            if (avgSinr < 10.0) { // Penalizar SINR bajo
                reliabilityScore *= (avgSinr / 10.0);
            }
        }
        reliabilityScore = std::max(0.0, std::min(100.0, reliabilityScore));
        
        // Escribir datos del flujo
        uint32_t cellId = g_imsiToCell[imsi];
        double distance = g_imsiDistance[imsi];
        std::string trafficType = isEmbb ? "eMBB" : "URLLC";
        
        flowOut << flowStat.first << "," << trafficType << "," << imsi << ","
                << cellId << "," << std::fixed << std::setprecision(2) << distance << ","
                << flowTuple.destinationAddress << ","
                << std::setprecision(2) << avgSinr << ","
                << chanMetrics.minSinr << "," << chanMetrics.maxSinr << ","
                << sinrStdDev << ","
                << fs.txPackets << "," << fs.rxPackets << "," << lostPackets << ","
                << std::setprecision(4) << packetLossRatio << ","
                << std::setprecision(3) << throughput << ","
                << std::setprecision(3) << meanDelay << ","
                << std::setprecision(3) << meanJitter << ","
                << std::setprecision(1) << qoeScore << ","
                << reliabilityScore << ",2\n"; // Numerología 2
        
        // Actualizar estadísticas por celda
        CellSummary& summary = cellSummaries[cellId];
        summary.totalThroughput += throughput;
        summary.totalTx += fs.txPackets;
        summary.totalRx += fs.rxPackets;
        summary.totalLost += lostPackets;
        summary.totalSinr += avgSinr;
        summary.sinrSamples++;
        summary.qoe.totalDelay += meanDelay;
        summary.qoe.totalJitter += meanJitter;
        summary.qoe.totalPackets += fs.rxPackets;
        summary.qoe.sumThroughput += throughput;
        summary.qoe.flows++;
        
        totalSystemThroughput += throughput;
    }
    
    flowOut.close();
    
    // ==================== Estadísticas por celda ===========================
    std::string cellFile = outputDir + "/cell_stats_optimized_" + std::to_string(numCells) +
                      "cell.csv";
    std::ofstream cellOut(cellFile);
    cellOut << "CellId,NumUEs,TotalThroughput(Mbps),SpectralEfficiency(bps/Hz),"
            << "TxPackets,RxPackets,LostPackets,PacketLossRatio(%),"
            << "AvgSINR(dB),AvgDelay(ms),AvgJitter(ms),"
            << "CellQoEScore,CellReliability(%),LoadBalance(%)\n";
    
    double maxCellThroughput = 0.0;
    for (const auto& cellStat : cellSummaries) {
        maxCellThroughput = std::max(maxCellThroughput, cellStat.second.totalThroughput);
    }
    
    for (uint32_t cellId = 0; cellId < numCells; cellId++) {
        const CellSummary& summary = cellSummaries[cellId];
        
        double packetLossRatio = (summary.totalTx > 0) ? 
                                (100.0 * summary.totalLost / summary.totalTx) : 0.0;
        double avgSinr = (summary.sinrSamples > 0) ? 
                        summary.totalSinr / summary.sinrSamples : 0.0;
        double avgDelay = (summary.qoe.flows > 0) ? 
                         summary.qoe.totalDelay / summary.qoe.flows : 0.0;
        double avgJitter = (summary.qoe.flows > 0) ? 
                          summary.qoe.totalJitter / summary.qoe.flows : 0.0;
        
        // Eficiencia espectral (asumiendo 100 MHz de ancho de banda)
        double spectralEfficiency = (summary.totalThroughput * 1e6) / 100e6; // bps/Hz
        
        // QoE Score por celda
        double cellQoE = 100.0;
        if (avgDelay > 10.0) cellQoE *= (10.0 / avgDelay);
        if (packetLossRatio > 1.0) cellQoE *= (1.0 / packetLossRatio);
        if (avgSinr < 15.0) cellQoE *= (avgSinr / 15.0);
        cellQoE = std::max(0.0, std::min(100.0, cellQoE));
        
        // Reliability basada en pérdidas y SINR
        double reliability = 100.0 - packetLossRatio * 10.0;
        if (avgSinr < 10.0) reliability *= (avgSinr / 10.0);
        reliability = std::max(0.0, std::min(100.0, reliability));
        
        // Load Balance (distribución equitativa del throughput)
        double loadBalance = (maxCellThroughput > 0) ? 
                            (summary.totalThroughput / maxCellThroughput * 100.0) : 0.0;
        
        cellOut << cellId << "," << g_cellUeCount[cellId] << ","
                << std::fixed << std::setprecision(3) << summary.totalThroughput << ","
                << std::setprecision(2) << spectralEfficiency << ","
                << summary.totalTx << "," << summary.totalRx << "," << summary.totalLost << ","
                << std::setprecision(4) << packetLossRatio << ","
                << std::setprecision(2) << avgSinr << ","
                << std::setprecision(3) << avgDelay << ","
                << std::setprecision(3) << avgJitter << ","
                << std::setprecision(1) << cellQoE << ","
                << reliability << "," << loadBalance << "\n";
    }
    
    cellOut.close();
    
    // ==================== Estadísticas del sistema =========================
    std::string systemFile = outputDir + "/system_stats_optimized_" + std::to_string(numCells) +
                        "cell.csv";
    std::ofstream systemOut(systemFile);
    systemOut << "Metric,Value,Unit\n";
    systemOut << "TotalSystemThroughput," << std::fixed << std::setprecision(3) 
              << totalSystemThroughput << ",Mbps\n";
    systemOut << "AvgThroughputPerCell," << (totalSystemThroughput / numCells) << ",Mbps\n";
    systemOut << "AvgThroughputPerUE," << (totalSystemThroughput / numUEs) << ",Mbps\n";
    
    // Latencias promedio por tipo
    double avgUrllcDelay = (urllcFlows > 0) ? (totalUrllcDelay / urllcFlows) : 0.0;
    double avgEmbbDelay = (embbFlows > 0) ? (totalEmbbDelay / embbFlows) : 0.0;
    systemOut << "AvgURLLCDelay," << std::setprecision(3) << avgUrllcDelay << ",ms\n";
    systemOut << "AvgEmbbDelay," << avgEmbbDelay << ",ms\n";
    
    systemOut << "HandoverAttempts," << g_handoverAttempts << ",count\n";
    systemOut << "HandoverSuccess," << g_handoverSuccess << ",count\n";
    systemOut << "HandoverFailures," << g_handoverFailures << ",count\n";
    
    double handoverSuccessRate = (g_handoverAttempts > 0) ? 
                                (100.0 * g_handoverSuccess / g_handoverAttempts) : 0.0;
    systemOut << "HandoverSuccessRate," << std::setprecision(2) 
              << handoverSuccessRate << ",%\n";
    
    // Calcular eficiencia espectral del sistema
    double systemSpectralEff = (totalSystemThroughput * 1e6) / (100e6 * numCells);
    systemOut << "SystemSpectralEfficiency," << std::setprecision(3) 
              << systemSpectralEff << ",bps/Hz/cell\n";
    
    // Densidad de usuarios
    double totalArea = M_PI * std::pow(ISD * 1.2, 2) * numCells; // Aproximación
    double userDensity = numUEs / (totalArea * 1e-6); // usuarios/km²
    systemOut << "UserDensity," << std::setprecision(1) << userDensity << ",UE/km2\n";
    
    systemOut << "ScenarioType," << scenarioName << ",type\n";
    systemOut << "NumCells," << numCells << ",count\n";
    systemOut << "NumUEs," << numUEs << ",count\n";
    systemOut << "InterSiteDistance," << ISD << ",m\n";
    systemOut << "SimulationTime," << simTime << ",s\n";
    systemOut << "Numerology,2,30kHz_SCS\n";
    systemOut << "UeTxPower," << ueTxPower << ",dBm\n";
    systemOut << "PropagationModel," << propagationModel << ",type\n";
    
    systemOut.close();
    
    // ==================== Archivo de configuración =========================
    std::string configFile = outputDir + "/simulation_config_optimized_" + std::to_string(numCells) +
                        "cell.txt";
    std::ofstream configOut(configFile);
    configOut << "=== OPTIMIZACIONES MÍNIMAS APLICADAS ===\n";
    configOut << "CAMBIOS (solo los compatibles con ns-3.44):\n\n";
    configOut << "1. NUMEROLOGÍA:\n";
    configOut << "   • Original: 1 (15 kHz SCS)\n";
    configOut << "   • Optimizado: 2 (30 kHz SCS)\n";
    configOut << "   • Beneficio: TTI más corto → menor latencia\n\n";
    configOut << "2. POTENCIA UE:\n";
    configOut << "   • Añadido: " << ueTxPower << " dBm\n";
    configOut << "   • Beneficio: Mejor SINR → menos retransmisiones\n\n";
    configOut << "4. APLICACIONES URLLC:\n";
    configOut << "   • Intervalo más frecuente para numerología 2\n";
    configOut << "   • Denso: 0.5 ms, Disperso: 1 ms\n\n";
    configOut << "5. MODELO PROPAGACIÓN:\n";
    configOut << "   • Denso: UMa (vs UMa original)\n";
    configOut << "   • Beneficio: Menor variabilidad\n\n";
    configOut << "7. HANDOVER:\n";
    configOut << "   • ServingCellThreshold: 15 dB (vs 18)\n";
    configOut << "   • NeighbourCellOffset: 3 dB (vs 5)\n\n";
    configOut << "=== PARÁMETROS IGUALES AL ORIGINAL ===\n";
    configOut << "Número de celdas: " << numCells << "\n";
    configOut << "Número de UEs: " << numUEs << "\n";
    configOut << "Proporción eMBB: " << embbRatio << "\n";
    configOut << "Proporción URLLC: " << (1.0 - embbRatio) << "\n";
    configOut << "Escenario: " << (denseScenario ? "Denso urbano" : "Disperso suburbano") << "\n";
    configOut << "Distancia inter-sitio: " << ISD << " m\n";
    configOut << "Altura gNB: " << gnbHeight << " m\n";
    configOut << "Altura UE: " << ueHeight << " m\n";
    configOut << "Potencia Tx gNB: " << gnbTxPower << " dBm (IGUAL)\n";
    configOut << "Frecuencia: 3.5 GHz (FR1)\n";
    configOut << "Ancho de banda: 100 MHz\n";
    configOut << "Scheduler: " << scheduler << "\n";
    configOut << "Algoritmo HO: " << hoAlgorithm << "\n";
    configOut << "Tiempo simulación: " << simTime << " s\n";
    configOut << "Semilla RNG: " << rngSeed << "\n";
    configOut.close();
    
    // ==================== Resumen en consola ===============================
    std::cout << "\n========== SIMULACIÓN COMPLETADA - OPTIMIZACIONES MÍNIMAS ==========\n";
    std::cout << "Escenario: " << numCells << " celdas " 
              << (denseScenario ? "DENSO" : "DISPERSO") << "\n";
    std::cout << "Throughput total: " << std::fixed << std::setprecision(2) 
              << totalSystemThroughput << " Mbps\n";
    std::cout << "Throughput promedio/UE: " << (totalSystemThroughput / numUEs) << " Mbps\n";
    std::cout << "Latencia promedio eMBB: " << std::setprecision(3) 
              << avgEmbbDelay << " ms\n";
    std::cout << "Latencia promedio URLLC: " << std::setprecision(3) 
              << avgUrllcDelay << " ms\n";
    
    std::cout << "Eficiencia espectral: " << std::setprecision(3) 
              << systemSpectralEff << " bps/Hz/celda\n";
    std::cout << "Tasa éxito handover: " << std::setprecision(1) 
              << handoverSuccessRate << "%\n";
    
    std::cout << "\n=== CAMBIOS APLICADOS (compatibles) ===\n";
    std::cout << "✓ Numerología: 2 (30 kHz vs 15 kHz)\n";
    std::cout << "✓ Potencia UE: " << ueTxPower << " dBm\n";
    std::cout << "✓ URLLC: intervalo optimizado\n";
    std::cout << "✓ Propagación: " << propagationModel << "\n";
    std::cout << "✓ Handover: umbrales optimizados\n";
    
    std::cout << "\n=== ARCHIVOS GENERADOS ===\n";
    std::cout << "• " << flowFile << "\n";
    std::cout << "• " << cellFile << "\n";
    std::cout << "• " << systemFile << "\n";
    std::cout << "• " << configFile << "\n";
    std::cout << "====================================================================\n\n";
    
    Simulator::Destroy();
    return 0;
}

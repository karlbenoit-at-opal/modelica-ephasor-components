within OpalRT.GenUnits.GENROU;
class GENROU_ESST3A_PSS2A_WSIEG1
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU"));
  parameter Real P_gen = 900 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 200 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU"));
  //parameter Real ZSOURCE_IM = Xq_s "Machine source impedence" annotation(Dialog(tab = "GENROU"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU"));
  parameter Real Tdo_s = 0.3 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU"));
  parameter Real Tqo_p = 0.04 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU"));
  parameter Real Tqo_s = 0.4 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU"));
  parameter Real H = 2.6 "Inertia constant" annotation(Dialog(tab = "GENROU"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU"));
  parameter Real Xd = 0.67 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xq = 0.62 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xd_p = 0.3 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xq_p = 0.3 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xd_s = 0.01 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  //parameter Real Xq_s = Xd_s "q-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xl = 0.04 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real S1 = 0.1 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU"));
  parameter Real S12 = 0.2 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU"));
  // ESST3A parameters
  parameter Real TR_ex = 0.02 "(sec) regulator input filter time constant" annotation(Dialog(tab = "ESST3A"));
  parameter Real VIMAX_ex = 10 "(pu) Voltage regulator input maximum limit" annotation(Dialog(tab = "ESST3A"));
  parameter Real VIMIN_ex = -10 "(pu) Voltage regulator input minimum limit" annotation(Dialog(tab = "ESST3A"));
  parameter Real KM_ex = 0.02 "Forward gain constant of the inner loop field regulator" annotation(Dialog(tab = "ESST3A"));
  parameter Real TC_ex = 1 " lead time constant of voltage regulator (s)" annotation(Dialog(tab = "ESST3A"));
  parameter Real TB_ex = 0.1 " lag time constant of voltage regulator (s)" annotation(Dialog(tab = "ESST3A"));
  parameter Real KA_ex = 10 "(pu) voltage regulator gain" annotation(Dialog(tab = "ESST3A"));
  parameter Real TA_ex = 0.02 "(sec) regulator time constant" annotation(Dialog(tab = "ESST3A"));
  parameter Real VRMAX_ex = 10 "(pu) regulator output maximum limit" annotation(Dialog(tab = "ESST3A"));
  parameter Real VRMIN_ex = -10 "(pu) regulator output minimum limit" annotation(Dialog(tab = "ESST3A"));
  parameter Real KG_ex = 1 "Feedback gain constant of the inner loop field regulator" annotation(Dialog(tab = "ESST3A"));
  parameter Real KP_ex = 1 "Potential circuit gain coefficient" annotation(Dialog(tab = "ESST3A"));
  parameter Real KI_ex = 0.02 "Potential circuit gain coefficient" annotation(Dialog(tab = "ESST3A"));
  parameter Real VBMAX_ex = 10 "(pu) regulator output maximum limit" annotation(Dialog(tab = "ESST3A"));
  parameter Real KC_ex = 1 "Rectifier loading factor proportional to commutating reactance" annotation(Dialog(tab = "ESST3A"));
  parameter Real XL_ex = 0.02 "Reactance associated with potential source" annotation(Dialog(tab = "ESST3A"));
  parameter Real VGMAX_ex = 10 "(pu) regulator output maximum limit" annotation(Dialog(tab = "ESST3A"));
  parameter Real THETAP_ex = 0.52 "Potential circuit phase angle (degrees)" annotation(Dialog(tab = "ESST3A"));
  parameter Real TM_ex = 0.02 "Forward time constant of the inner loop field regulator" annotation(Dialog(tab = "ESST3A"));
  parameter Real VMMAX_ex = 10 "(pu) regulator output maximum limit" annotation(Dialog(tab = "ESST3A"));
  parameter Real VMMIN_ex = -10 "(pu) regulator output minimum limit" annotation(Dialog(tab = "ESST3A"));
  //PSS2A parameters
  parameter Real TW1_pss = 10 ">0" annotation(Dialog(tab = "PSS2A"));
  parameter Real TW2_pss = 10 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real T6_pss = 0 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real TW3_pss = 10 ">0" annotation(Dialog(tab = "PSS2A"));
  parameter Real TW4_pss = 0 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real T7_pss = 10 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real KS2_pss = 1.13 annotation(Dialog(tab = "PSS2A"));
  parameter Real KS3_pss = 1 annotation(Dialog(tab = "PSS2A"));
  parameter Real T8_pss = 0.3;
  parameter Real T9_pss = 0.15 ">0" annotation(Dialog(tab = "PSS2A"));
  parameter Real KS1_pss = 20 annotation(Dialog(tab = "PSS2A"));
  parameter Real T1_pss = 0.16 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real T2_pss = 0.02 annotation(Dialog(tab = "PSS2A"));
  parameter Real T3_pss = 0.16 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A"));
  parameter Real T4_pss = 0.02 annotation(Dialog(tab = "PSS2A"));
  parameter Real VSTMAX_pss = 0.2 annotation(Dialog(tab = "PSS2A"));
  parameter Real VSTMIN_pss = -0.066 annotation(Dialog(tab = "PSS2A"));
  parameter Real M0_pss = 1 annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M1_pss = 0 "currently disabled" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M2_pss = 1 annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M3_pss = 0 "currently disabled" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M4_pss = 8 ">= 0, To bypass Ramp Tracking Filter: set M = N = 0" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real M5_pss = 8 ">= 0, M*N <= 8" annotation(Dialog(tab = "PSS2A", group = "ICONs"));
  parameter Real y_start = 0 "Initial or guess value of output (= state)";
  parameter Real noVOEL(fixed = false, start = 1);
  parameter Real noVUEL(fixed = false, start = 1);
  // WSIEG1 parameters
  parameter Real K_tg = 20 annotation(Dialog(tab = "WSIG1"));
  parameter Real T1_tg = 0.5 "(sec)" annotation(Dialog(tab = "WSIG1"));
  parameter Real T2_tg = 1 "(sec)" annotation(Dialog(tab = "WSIG1"));
  parameter Real T3_tg = 1 "(>0)(sec)" annotation(Dialog(tab = "WSIG1"));
  parameter Real Uo_tg = 0.1 "(pu/sec)" annotation(Dialog(tab = "WSIG1"));
  parameter Real Uc_tg = -0.2 "(<0)(pu/sec)" annotation(Dialog(tab = "WSIG1"));
  parameter Real PMAX_tg = 1 "(pu on machine MVA rating)" annotation(Dialog(tab = "WSIG1"));
  parameter Real PMIN_tg = 0 "(pu on machine MVA rating)" annotation(Dialog(tab = "WSIG1"));
  parameter Real T4_tg = 0.4 "(sec)" annotation(Dialog(tab = "WSIG1"));
  parameter Real K1_tg = 0.2 annotation(Dialog(tab = "WSIG1"));
  parameter Real K2_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real T5_tg = 7 "(sec)" annotation(Dialog(tab = "WSIG1"));
  parameter Real K3_tg = 0.1 annotation(Dialog(tab = "WSIG1"));
  parameter Real K4_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real T6_tg = 0.6 "(sec)" annotation(Dialog(tab = "WSIG1"));
  parameter Real K5_tg = 0.2 annotation(Dialog(tab = "WSIG1"));
  parameter Real K6_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real T7_tg = 0.3 "(sec)" annotation(Dialog(tab = "WSIG1"));
  parameter Real K7_tg = 0.1 annotation(Dialog(tab = "WSIG1"));
  parameter Real K8_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real db1_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real err_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real db2_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real GV1_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real PGV1_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real GV2_tg = 99 annotation(Dialog(tab = "WSIG1"));
  parameter Real PGV2_tg = 99 annotation(Dialog(tab = "WSIG1"));
  parameter Real GV3_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real PGV3_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real GV4_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real PGV4_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real GV5_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real PGV5_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real IBLOCK_tg = 0 annotation(Dialog(tab = "WSIG1"));
  parameter Real JBUS = 0 "Bus Identifier (NOT USED)" annotation(Dialog(tab = "WSIG1"));
  parameter Real M = 0 "Machine Identifier (NOT USED)" annotation(Dialog(tab = "WSIG1"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {60, 18}, extent = {{-24, -24}, {24, 24}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {62, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-96, 68}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, -26}, extent = {{-8.5, -8.5}, {8.5, 8.5}}, rotation = 0), iconTransformation(origin = {102, -60}, extent = {{-11.5, -11.5}, {11.5, 11.5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESST3A esst3a1(TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, KM = KM_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KG = KG_ex, KP = KP_ex, KI = KI_ex, VBMAX = VBMAX_ex, KC = KC_ex, XL = XL_ex, VGMAX = VGMAX_ex, THETAP = THETAP_ex, TM = TM_ex, VMMAX = VMMAX_ex, VMMIN = VMMIN_ex) annotation(Placement(visible = true, transformation(origin = {-2, 40}, extent = {{-24.9063, -24.9063}, {24.9063, 24.9063}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = noVOEL) annotation(Placement(visible = true, transformation(origin = {-56, 40}, extent = {{-7.625, -7.625}, {7.625, 7.625}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2A pss2a1(TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M2 = M2_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = 0) annotation(Placement(visible = true, transformation(origin = {-84, 34}, extent = {{-18, -18}, {18, 18}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.WSIEG1 wsieg11(IBUS = IBUS, ID = M_ID, K = K_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, Uo = Uo_tg, Uc = Uc_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, T4 = T4_tg, K1 = K1_tg, K2 = K2_tg, T5 = T5_tg, K3 = K3_tg, K4 = K4_tg, T6 = T6_tg, K5 = K5_tg, K6 = K6_tg, T7 = T7_tg, K7 = K7_tg, K8 = K8_tg, db1 = db1_tg, err = err_tg, db2 = db2_tg, GV1 = GV1_tg, PGV1 = PGV1_tg, GV2 = GV2_tg, PGV2 = PGV2_tg, GV3 = GV3_tg, PGV3 = PGV3_tg, GV4 = GV4_tg, PGV4 = PGV4_tg, GV5 = GV5_tg, PGV5 = PGV5_tg, IBLOCK = IBLOCK_tg, JBUS = JBUS, M = M) annotation(Placement(visible = true, transformation(origin = {-4, -50}, extent = {{-24.25, -24.25}, {24.25, 24.25}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = noVUEL) annotation(Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-90, 2}, extent = {{-7.5, -7.5}, {7.5, 7.5}}, rotation = 0), iconTransformation(origin = {-96, -62}, extent = {{-11.625, -11.625}, {11.625, 11.625}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-90, -24}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {94, 67}, extent = {{12, -12}, {-12, 12}}, rotation = 0)));
initial equation
  noVOEL = 0;
  noVUEL = -Modelica.Constants.inf;
equation
  connect(genrou1.VI, wsieg11.VI) annotation(Line(points = {{84, 18}, {100, 18}, {100, -93.133}, {-42.4893, -93.133}, {-42.4893, -49.7854}, {-30.0429, -49.7854}, {-30.0429, -49.7854}}, color = {0, 0, 127}));
  connect(genrou1.VI, pss2a1.VI2) annotation(Line(points = {{84, 18}, {100, 18}, {100, -93.133}, {-115.021, -93.133}, {-115.021, 26.6094}, {-102.146, 26.6094}, {-102.146, 26.6094}}, color = {0, 0, 127}));
  connect(genrou1.AccPower, pss2a1.PSS_AUX2[2]) annotation(Line(points = {{84, 11.76}, {96.5665, 11.76}, {96.5665, -87.5536}, {-109.013, -87.5536}, {-109.013, 22.3176}, {-101.717, 22.3176}, {-101.717, 22.3176}}, color = {0, 0, 127}));
  connect(pss2a1.VI2, pss2a1.VI) annotation(Line(points = {{-102, 26.8}, {-108.155, 26.8}, {-108.155, 38.1974}, {-102.146, 38.1974}, {-102.146, 38.1974}}, color = {0, 0, 127}));
  connect(pss2a1.PSS_AUX2, pss2a1.PSS_AUX) annotation(Line(points = {{-102, 23.2}, {-112.017, 23.2}, {-112.017, 33.9056}, {-102.575, 33.9056}, {-102.575, 33.9056}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, pss2a1.PSS_AUX2[1]) annotation(Line(points = {{84, -1.2}, {89.6996, -1.2}, {89.6996, -76.3948}, {-109.442, -76.3948}, {-109.442, 21.8884}, {-102.146, 21.8884}, {-102.146, 21.8884}}, color = {0, 0, 127}));
  connect(esst3a1.VI, genrou1.VI) annotation(Line(points = {{22.9063, 54.9438}, {100.429, 54.9438}, {100.429, 17.5966}, {85.4077, 17.5966}, {85.4077, 17.5966}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, wsieg11.MBASE) annotation(Line(points = {{84, 5.04}, {93.133, 5.04}, {93.133, -80.6867}, {-38.1974, -80.6867}, {-38.1974, -60.0858}, {-30.0429, -60.0858}, {-30.0429, -60.0858}}, color = {0, 0, 127}));
  connect(dGREF, wsieg11.dGREF) annotation(Line(points = {{-90, -24}, {-59.2275, -24}, {-59.2275, -30.9013}, {-27.897, -30.9013}, {-27.897, -30.9013}}));
  connect(dVREF, esst3a1.dVREF) annotation(Line(points = {{-90, 2}, {-35.6223, 2}, {-35.6223, 23.6052}, {-27.4678, 23.6052}, {-27.4678, 23.6052}}));
  connect(constant2.y, esst3a1.VUEL) annotation(Line(points = {{-52.3, 60}, {-38.9671, 60}, {-38.9671, 44.6009}, {-27.6995, 44.6009}, {-27.6995, 44.6009}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, wsieg11.SLIP) annotation(Line(points = {{84, -1.2}, {89.6714, -1.2}, {89.6714, -76.5258}, {-33.3333, -76.5258}, {-33.3333, -70.4225}, {-29.5775, -70.4225}, {-29.5775, -70.4225}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, wsieg11.PMECH0) annotation(Line(points = {{36, -1.2}, {28.6385, -1.2}, {28.6385, -35.6808}, {22.5352, -35.6808}, {22.5352, -35.6808}}, color = {0, 0, 127}));
  connect(wsieg11.PMECH, genrou1.PMECH) annotation(Line(points = {{20.25, -30.6}, {25.8216, -30.6}, {25.8216, 3.75587}, {34.7418, 3.75587}, {34.7418, 3.75587}}, color = {0, 0, 127}));
  connect(pss2a1.VOTHSG, esst3a1.VOTHSG) annotation(Line(points = {{-66, 23.2}, {-40.3756, 23.2}, {-40.3756, 31.4554}, {-28.6385, 31.4554}, {-28.6385, 31.4554}}, color = {0, 0, 127}));
  connect(const.y, esst3a1.VOEL) annotation(Line(points = {{-47.6125, 40}, {-28.6385, 40}, {-28.6385, 38.4977}, {-28.6385, 38.4977}}, color = {0, 0, 127}));
  connect(genrou1.XADIFD, esst3a1.XADIFD) annotation(Line(points = {{84, 30}, {92.9577, 30}, {92.9577, 91.5493}, {-36.1502, 91.5493}, {-36.1502, 51.6432}, {-28.169, 51.6432}, {-28.169, 51.6432}}, color = {0, 0, 127}));
  connect(genrou1.EX_AUX, esst3a1.EX_AUX) annotation(Line(points = {{35.52, 37.2}, {25.3521, 37.2}, {25.3521, 35.6808}, {25.3521, 35.6808}}, color = {0, 0, 127}));
  connect(genrou1.ETERM0, esst3a1.ETERM0) annotation(Line(points = {{35.52, 30}, {24.8826, 30}, {24.8826, 30.0469}, {24.8826, 30.0469}}, color = {0, 0, 127}));
  connect(esst3a1.EFD, genrou1.EFD) annotation(Line(points = {{22.9063, 25.0562}, {34.7418, 25.0562}, {34.7418, 25.3521}, {34.7418, 25.3521}}, color = {0, 0, 127}));
  connect(genrou1.EFD0, esst3a1.EFD0) annotation(Line(points = {{35.52, 18}, {28.169, 18}, {28.169, 20.1878}, {24.4131, 20.1878}, {24.4131, 20.1878}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{62, 76}, {60.0939, 76}, {60.0939, 44.1315}, {60.0939, 44.1315}}));
  connect(genrou1.p, bus0) annotation(Line(points = {{60, -6}, {60.5634, -6}, {60.5634, -21.1268}, {60.5634, -21.1268}}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {-70.8096, 84.7206}, extent = {{-27.252, 13.569}, {169.327, -183.01}}), Text(origin = {-48.8941, -63.7106}, extent = {{-31.93, 18.81}, {14.1319, -11.7568}}, textString = "dVREF"), Text(origin = {-60.9276, 59.6372}, extent = {{-29.53, 19.27}, {21.7763, -1.48209}}, textString = "TRIP"), Text(origin = {63.8582, -57.3586}, extent = {{-14.6, 11.97}, {20.3013, -15.8469}}, textString = "bus0"), Text(origin = {-12.88, 14.9159}, extent = {{-80.62, 26.15}, {107.99, -51.36}}, textString = "GENROU_ESST3A_PSS2A_WSIEG1"), Text(origin = {62.4397, 63.8479}, extent = {{-31.92, 14.55}, {19.7134, -6.0993}}, textString = "dGREF")}));
end GENROU_ESST3A_PSS2A_WSIEG1;

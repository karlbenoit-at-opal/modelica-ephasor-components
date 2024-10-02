within OpalRT.GenUnits.GENROU;
class GENROU_ESST4B_WSIEG1
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "General"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // ESST4B Parameters
  parameter Real TR_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real KPR_ex = 1 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real KIR_ex = 0.03 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real VRMAX_ex = 10 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real VRMIN_ex = -10 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real TA_ex = 0.2 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real KPM_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real KIM_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real VMMAX_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real VMMIN_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real KG_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real KP_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real KI_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real VBMAX_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real KC_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real XL_ex = 0.02 annotation(Dialog(tab = "ESST4B Parameters"));
  parameter Real THETAP_ex = 0.52 annotation(Dialog(tab = "ESST4B Parameters"));
  // WSIEG1 Parameters
  parameter Real JBUS_tg = 0 "Bus Identifier (NOT USED)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real M_tg = 0 "Machine Identifier (NOT USED)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real K_tg = 15 annotation(Dialog(tab = "WSIEG1"));
  parameter Real T1_tg = 1.5 "(sec)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real T2_tg = 0.3 "(sec)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real T3_tg = 0.02 "(>0)(sec)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real Uo_tg = 0.1 "(pu/sec)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real Uc_tg = -0.5 "(<0)(pu/sec)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real PMAX_tg = 0.5 "(pu on machine MVA rating)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real PMIN_tg = 0 "(pu on machine MVA rating)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real T4_tg = 0.1 "(sec)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real K1_tg = 0.1 annotation(Dialog(tab = "WSIEG1"));
  parameter Real K2_tg = 0.1 annotation(Dialog(tab = "WSIEG1"));
  parameter Real T5_tg = 0.2 "(sec)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real K3_tg = 0.5 annotation(Dialog(tab = "WSIEG1"));
  parameter Real K4_tg = 0.1 annotation(Dialog(tab = "WSIEG1"));
  parameter Real T6_tg = 0.1 "(sec)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real K5_tg = 0.6 annotation(Dialog(tab = "WSIEG1"));
  parameter Real K6_tg = 0.2 annotation(Dialog(tab = "WSIEG1"));
  parameter Real T7_tg = 0.3 "(sec)" annotation(Dialog(tab = "WSIEG1"));
  parameter Real K7_tg = 0.2 annotation(Dialog(tab = "WSIEG1"));
  parameter Real K8_tg = 0.6 annotation(Dialog(tab = "WSIEG1"));
  parameter Real db1_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real err_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real db2_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real GV1_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real PGV1_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real GV2_tg = 99 annotation(Dialog(tab = "WSIEG1"));
  parameter Real PGV2_tg = 99 annotation(Dialog(tab = "WSIEG1"));
  parameter Real GV3_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real PGV3_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real GV4_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real PGV4_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real GV5_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real PGV5_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  parameter Real IBLOCK_tg = 0 annotation(Dialog(tab = "WSIEG1"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 20}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESST4B esst4b1(TR = TR_ex, KPR = KPR_ex, KIR = KIR_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, TA = TA_ex, KPM = KPM_ex, KIM = KIM_ex, VMMAX = VMMAX_ex, VMMIN = VMMIN_ex, KG = KG_ex, KP = KP_ex, KI = KI_ex, VBMAX = VBMAX_ex, KC = KC_ex, XL = XL_ex, THETAP = THETAP_ex) annotation(Placement(visible = true, transformation(origin = {-20, 40}, extent = {{-21.25, -21.25}, {21.25, 21.25}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const2(k = 0) annotation(Placement(visible = true, transformation(origin = {-60, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  parameter Real noVOEL(fixed = false, start = 1);
  Modelica.Blocks.Sources.Constant const1(k = noVOEL) annotation(Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.WSIEG1 wsieg11(IBUS = IBUS, ID = M_ID, K = K_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, Uo = Uo_tg, Uc = Uc_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, T4 = T4_tg, K1 = K1_tg, K2 = K2_tg, T5 = T5_tg, K3 = K3_tg, K4 = K4_tg, T6 = T6_tg, K5 = K5_tg, K6 = K6_tg, T7 = T7_tg, K7 = K7_tg, K8 = K8_tg, db1 = db1_tg, err = err_tg, db2 = db2_tg, GV1 = GV1_tg, PGV1 = PGV1_tg, GV2 = GV2_tg, PGV2 = PGV2_tg, GV3 = GV3_tg, PGV3 = PGV3_tg, GV4 = GV4_tg, PGV4 = PGV4_tg, GV5 = GV5_tg, PGV5 = PGV5_tg, IBLOCK = IBLOCK_tg, JBUS = 0, M = 0) annotation(Placement(visible = true, transformation(origin = {-20, -40}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-80, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
initial equation
  noVOEL = Modelica.Constants.inf;
equation
  connect(esst4b1.VI, genrou1.VI) annotation(Line(points = {{1.25, 52.75}, {101.288, 52.75}, {101.288, 19.7425}, {63.9485, 19.7425}, {63.9485, 19.7425}}, color = {0, 0, 127}));
  connect(genrou1.VI, wsieg11.VI) annotation(Line(points = {{65, 20}, {101.717, 20}, {101.717, -78.5408}, {-57.5107, -78.5408}, {-57.5107, -41.2017}, {-45.4936, -41.2017}, {-45.4936, -41.2017}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, wsieg11.MBASE) annotation(Line(points = {{65, 6.5}, {100, 6.5}, {100, -74.2489}, {-54.9356, -74.2489}, {-54.9356, -49.7854}, {-43.7768, -49.7854}, {-43.7768, -49.7854}}, color = {0, 0, 127}));
  connect(wsieg11.dGREF, dGREF) annotation(Line(points = {{-45, -20}, {-78.1116, -20}, {-78.1116, -33.9056}, {-78.1116, -33.9056}}, color = {0, 0, 127}));
  connect(dVREF, esst4b1.dVREF) annotation(Line(points = {{-80, 40}, {-75.9657, 40}, {-75.9657, 26.6094}, {-41.2017, 26.6094}, {-41.2017, 26.6094}}));
  connect(wsieg11.SLIP, genrou1.SLIP) annotation(Line(points = {{-45, -60}, {-52.5745, -60.271}, {-52.5745, -68.5637}, {98.916, -68.5637}, {98.916, 0}, {65, 0}, {65, 0}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, wsieg11.PMECH0) annotation(Line(points = {{15, 0}, {11.6531, 0}, {11.6531, -24.6612}, {6.77507, -24.6612}, {6.77507, -24.6612}}, color = {0, 0, 127}));
  connect(genrou1.PMECH, wsieg11.PMECH) annotation(Line(points = {{15, 5}, {8.67209, 5}, {8.67209, -19.7832}, {6.50407, -19.7832}, {6.50407, -19.7832}}, color = {0, 0, 127}));
  connect(esst4b1.VOEL, const1.y) annotation(Line(points = {{-41.25, 38.3}, {-50.4065, 38.3}, {-50.4065, 59.8916}, {-55.5556, 59.8916}, {-55.5556, 59.8916}}, color = {0, 0, 127}));
  connect(esst4b1.VOTHSG, const2.y) annotation(Line(points = {{-41.25, 32.35}, {-51.2195, 32.35}, {-51.2195, 40.1084}, {-54.2005, 40.1084}, {-54.2005, 40.1084}}, color = {0, 0, 127}));
  connect(esst4b1.VUEL, const2.y) annotation(Line(points = {{-41.25, 44.25}, {-51.2195, 44.25}, {-51.2195, 39.8374}, {-54.2005, 39.8374}, {-54.2005, 39.8374}}, color = {0, 0, 127}));
  connect(esst4b1.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-41.25, 50.625}, {-47.9675, 50.625}, {-47.9675, 75.3388}, {71.8157, 75.3388}, {71.8157, 32.7913}, {65.3117, 32.7913}, {65.3117, 32.7913}}, color = {0, 0, 127}));
  connect(genrou1.TRIP, TRIP) annotation(Line(points = {{40, 45}, {39.8374, 45}, {39.8374, 52.0325}, {39.8374, 52.0325}}, color = {0, 0, 127}));
  connect(esst4b1.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{1.25, 37.025}, {7.04607, 37.025}, {7.04607, 40.1084}, {14.0921, 40.1084}, {14.0921, 40.1084}}, color = {0, 0, 127}));
  connect(esst4b1.ETERM0, genrou1.ETERM0) annotation(Line(points = {{1.25, 31.5}, {7.58808, 31.5}, {7.58808, 32.7913}, {15.4472, 32.7913}, {15.4472, 32.7913}}, color = {0, 0, 127}));
  connect(genrou1.EFD, esst4b1.EFD) annotation(Line(points = {{15, 27}, {2.43902, 27}, {2.43902, 26.8293}, {2.43902, 26.8293}}, color = {0, 0, 127}));
  connect(esst4b1.EFD0, genrou1.EFD0) annotation(Line(points = {{1.25, 23}, {7.58808, 23}, {7.58808, 19.7832}, {13.5501, 19.7832}, {13.5501, 19.7832}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{80, -20}, {39.8374, -20}, {39.8374, -4.06504}, {39.8374, -4.06504}}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.406504, -3.38753}, extent = {{-100.136, 88.4824}, {98.5095, -62.1951}}), Text(origin = {-34.2821, 43.0943}, extent = {{-54.0662, 26.2881}, {127.508, -54.2014}}, textString = "GENROU_ESST4B_WSIEG1"), Text(origin = {-72.353, 59.6174}, extent = {{-14.63, 10.3}, {12.462, -8.94499}}, textString = "TRIP"), Text(origin = {-71.592, -36.915}, extent = {{-14.63, 10.3}, {22.4891, -16.2621}}, textString = "dGREF"), Text(origin = {69.82, -36.968}, extent = {{-5.41591, 4.06694}, {14.63, -10.3}}, textString = "PIN"), Text(origin = {-71.326, 0.751057}, extent = {{-14.63, 10.3}, {21.1361, -12.463}}, textString = "dVREF")}));
end GENROU_ESST4B_WSIEG1;

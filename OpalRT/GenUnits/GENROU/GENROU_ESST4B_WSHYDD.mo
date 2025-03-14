within OpalRT.GenUnits.GENROU;
class GENROU_ESST4B_WSHYDD
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
  // WSHYDD Parameters
  parameter Real db1_tg = 0 annotation(Dialog(tab = "WSHYDD"));
  parameter Real err_tg = 0 annotation(Dialog(tab = "WSHYDD"));
  parameter Real Td_tg = 1 "(sec)" annotation(Dialog(tab = "WSHYDD"));
  parameter Real K1_tg = 0.8 annotation(Dialog(tab = "WSHYDD"));
  parameter Real Tf_tg = 0.1 "(sec)" annotation(Dialog(tab = "WSHYDD"));
  parameter Real KD_tg = 0.8 annotation(Dialog(tab = "WSHYDD"));
  parameter Real KP_tg = 0.25 annotation(Dialog(tab = "WSHYDD"));
  parameter Real R_tg = 0.04 annotation(Dialog(tab = "WSHYDD"));
  parameter Real Tt_tg = 0.4 annotation(Dialog(tab = "WSHYDD"));
  parameter Real KG_tg = 2.0 annotation(Dialog(tab = "WSHYDD"));
  parameter Real TP_tg = 0.2 "(sec)" annotation(Dialog(tab = "WSHYDD"));
  parameter Real VELopen_tg = 0.007 "(>0)" annotation(Dialog(tab = "WSHYDD"));
  parameter Real VELclose_tg = 0.002 "(>0)" annotation(Dialog(tab = "WSHYDD"));
  parameter Real PMAX_tg = 1.0 annotation(Dialog(tab = "WSHYDD"));
  parameter Real PMIN_tg = 0 annotation(Dialog(tab = "WSHYDD"));
  parameter Real db2_tg = 0 annotation(Dialog(tab = "WSHYDD"));
  parameter Real GV1_tg = 0 annotation(Dialog(tab = "WSHYDD"));
  parameter Real PGV1_tg = 0 annotation(Dialog(tab = "WSHYDD"));
  parameter Real GV2_tg = 0.6 annotation(Dialog(tab = "WSHYDD"));
  parameter Real PGV2_tg = 0.7 annotation(Dialog(tab = "WSHYDD"));
  parameter Real GV3_tg = 0.7 annotation(Dialog(tab = "WSHYDD"));
  parameter Real PGV3_tg = 0.82 annotation(Dialog(tab = "WSHYDD"));
  parameter Real GV4_tg = 0.80 annotation(Dialog(tab = "WSHYDD"));
  parameter Real PGV4_tg = 0.90 annotation(Dialog(tab = "WSHYDD"));
  parameter Real GV5_tg = 0.90 annotation(Dialog(tab = "WSHYDD"));
  parameter Real PGV5_tg = 0.95 annotation(Dialog(tab = "WSHYDD"));
  parameter Real Aturb_tg = -1 annotation(Dialog(tab = "WSHYDD"));
  parameter Real Bturb_tg = 0.5 "(>0)" annotation(Dialog(tab = "WSHYDD"));
  parameter Real Tturb_tg = 0.9 "(>0)(sec)" annotation(Dialog(tab = "WSHYDD"));
  parameter Real TRATE_tg = 900 annotation(Dialog(tab = "WSHYDD"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 20}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESST4B esst4b1(TR = TR_ex, KPR = KPR_ex, KIR = KIR_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, TA = TA_ex, KPM = KPM_ex, KIM = KIM_ex, VMMAX = VMMAX_ex, VMMIN = VMMIN_ex, KG = KG_ex, KP = KP_ex, KI = KI_ex, VBMAX = VBMAX_ex, KC = KC_ex, XL = XL_ex, THETAP = THETAP_ex) annotation(Placement(visible = true, transformation(origin = {-20, 40}, extent = {{-21.25, -21.25}, {21.25, 21.25}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.WSHYDD wshydd1(db1 = db1_tg, err = err_tg, Td = Td_tg, K1 = K1_tg, Tf = Tf_tg, KD = KD_tg, KP = KP_tg, R = R_tg, Tt = Tt_tg, KG = KG_tg, TP = TP_tg, VELopen = VELopen_tg, VELclose = VELclose_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, db2 = db2_tg, GV1 = GV1_tg, PGV1 = PGV1_tg, GV2 = GV2_tg, PGV2 = PGV2_tg, GV3 = GV3_tg, PGV3 = PGV3_tg, GV4 = GV4_tg, PGV4 = PGV4_tg, GV5 = GV5_tg, PGV5 = PGV5_tg, Aturb = Aturb_tg, Bturb = Bturb_tg, Tturb = Tturb_tg, TRATE = TRATE_tg, IBUS = IBUS, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-20, -40}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = noVOEL) annotation(Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const2(k = 0) annotation(Placement(visible = true, transformation(origin = {-60, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Real noVOEL(fixed = false, start = 1);
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-92, 24}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-92, -20}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
initial equation
  noVOEL = Modelica.Constants.inf;
equation
  connect(esst4b1.VI, genrou1.VI) annotation(Line(points = {{1.25, 52.75}, {96.9957, 52.75}, {96.9957, 20.1717}, {66.0944, 20.1717}, {66.0944, 20.1717}}, color = {0, 0, 127}));
  connect(genrou1.VI, wshydd1.VI) annotation(Line(points = {{65, 20}, {97.4249, 20}, {97.4249, -75.9657}, {-56.2232, -75.9657}, {-56.2232, -39.485}, {-45.9227, -39.485}, {-45.9227, -39.485}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, wshydd1.MBASE) annotation(Line(points = {{65, 6.5}, {94.8498, 6.5}, {94.8498, -73.8197}, {-52.3605, -73.8197}, {-52.3605, -50.2146}, {-46.3519, -50.2146}, {-46.3519, -50.2146}}, color = {0, 0, 127}));
  connect(dGREF, wshydd1.dGREF) annotation(Line(points = {{-92, -20}, {-45.4936, -20}, {-45.4936, -21.8884}, {-45.4936, -21.8884}}));
  connect(dVREF, esst4b1.dVREF) annotation(Line(points = {{-92, 24}, {-63.0901, 24}, {-63.0901, 26.6094}, {-42.0601, 26.6094}, {-42.0601, 26.6094}}));
  connect(wshydd1.SLIP, genrou1.SLIP) annotation(Line(points = {{-45, -60}, {-51.4905, -60}, {-51.4905, -72.0867}, {92.4119, -72.0867}, {92.4119, -0.271003}, {65.5827, -0.271003}, {65.5827, -0.271003}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, wshydd1.PMECH0) annotation(Line(points = {{15, 0}, {11.3821, 0}, {11.3821, -24.6612}, {7.31707, -24.6612}, {7.31707, -24.6612}, {7.31707, -24.6612}}, color = {0, 0, 127}));
  connect(wshydd1.PMECH, genrou1.PMECH) annotation(Line(points = {{5, -20}, {8.13008, -20}, {8.13008, 4.87805}, {13.8211, 4.87805}, {13.8211, 5.42005}, {13.8211, 5.42005}}, color = {0, 0, 127}));
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
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.406504, -3.38753}, extent = {{-100.136, 88.4824}, {98.5095, -62.1951}}), Text(origin = {-34.28, 43.09}, extent = {{-54.07, 26.29}, {127.51, -54.2}}, textString = "GENROU_ESST4B_WSHYDD"), Text(origin = {-72.353, 59.6174}, extent = {{-14.63, 10.3}, {12.462, -8.94499}}, textString = "TRIP"), Text(origin = {-71.592, -36.915}, extent = {{-14.63, 10.3}, {22.4891, -16.2621}}, textString = "dGREF"), Text(origin = {69.82, -36.968}, extent = {{-5.41591, 4.06694}, {14.63, -10.3}}, textString = "PIN"), Text(origin = {-71.326, 0.751057}, extent = {{-14.63, 10.3}, {21.1361, -12.463}}, textString = "dVREF")}));
end GENROU_ESST4B_WSHYDD;

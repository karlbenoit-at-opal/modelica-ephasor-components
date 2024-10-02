within OpalRT.GenUnits.GENROU;
class GENROU_ESDC2A_IEEEST_IEEEG1
  parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 900 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 200 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  //parameter Real ZSOURCE_IM = Xq_s "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.3 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.04 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.4 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 2.6 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.67 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.62 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.3 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.3 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.01 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  //parameter Real Xq_s = Xd_s "q-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.04 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.1 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.2 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // ESDC2A parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real TF1_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real Switch_ex = 0 annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "ESDC2A parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "ESDC2A parameters"));
  // IEEEST Parameters
  parameter Real A1_pss = 2 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A2_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A3_pss = 2 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A4_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A5_pss = 2 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A6_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T1_pss = 0.03 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T2_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T3_pss = 0.02 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T4_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T5_pss = 0.2 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T6_pss = 0.1 "(>0)(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real KS_pss = -5 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMAX_pss = 6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMIN_pss = -6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCU_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCL_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  // IEEEST ICONs
  parameter Real M0_pss = 1 "Stabilizer input code" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  parameter Real M1_pss = 1 "IB, remote bus number" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  // IEEEG1 Parameters
  //parameter Integer IBUS = 100 "Located Bus No.";
  //parameter String TG_ID = M_ID"M1" "Machine Identifier";
  parameter Real JBUS_tg = 0 "Bus Identifier (NOT USED)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real M_tg = 0 "Machine Identifier (NOT USED)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K_tg = 15 annotation(Dialog(tab = "IEEEG1"));
  parameter Real T1_tg = 1.5 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real T2_tg = 0.3 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real T3_tg = 0.02 "(>0)(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real Uo_tg = 0.1 "(pu/sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real Uc_tg = -0.5 "(<0)(pu/sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real PMAX_tg = 0.5 "(pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real PMIN_tg = 0 "(pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real T4_tg = 0.1 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K1_tg = 0.1 annotation(Dialog(tab = "IEEEG1"));
  parameter Real K2_tg = 0.1 annotation(Dialog(tab = "IEEEG1"));
  parameter Real T5_tg = 0.2 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K3_tg = 0.5 annotation(Dialog(tab = "IEEEG1"));
  parameter Real K4_tg = 0.1 annotation(Dialog(tab = "IEEEG1"));
  parameter Real T6_tg = 0.1 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K5_tg = 0.6 annotation(Dialog(tab = "IEEEG1"));
  parameter Real K6_tg = 0.2 annotation(Dialog(tab = "IEEEG1"));
  parameter Real T7_tg = 0.3 "(sec)" annotation(Dialog(tab = "IEEEG1"));
  parameter Real K7_tg = 0.2 annotation(Dialog(tab = "IEEEG1"));
  parameter Real K8_tg = 0.6 annotation(Dialog(tab = "IEEEG1"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {60, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESDC2A esdc2a(TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {0, 14}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG1 ieeeg1(K = K_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, Uo = Uo_tg, Uc = Uc_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, T4 = T4_tg, K1 = K1_tg, K2 = K2_tg, T5 = T5_tg, K3 = K3_tg, K4 = K4_tg, T6 = T6_tg, K5 = K5_tg, K6 = K6_tg, T7 = T7_tg, K7 = K7_tg, K8 = K8_tg, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {4, -21}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-91, 20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.IEEEST ieeest1(ID = M_ID, A1 = A1_pss, A2 = A2_pss, A3 = A3_pss, A4 = A4_pss, A5 = A5_pss, A6 = A6_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, KS = KS_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M1 = 0) annotation(Placement(visible = true, transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, -28}, extent = {{-8.5, 8.5}, {8.5, -8.5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-9.5, -9.5}, {9.5, 9.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-53, -10}, extent = {{-2.5, -2.5}, {2.5, 2.5}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-9.5, -9.5}, {9.5, 9.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 20}, extent = {{-3.125, -3.125}, {3.125, 3.125}}, rotation = 0), iconTransformation(origin = {-99, 66}, extent = {{-9.125, -9.125}, {9.125, 9.125}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-52, 8}, extent = {{-2.5, -2.5}, {2.5, 2.5}}, rotation = 0), iconTransformation(origin = {99, 64}, extent = {{9.5, -9.5}, {-9.5, 9.5}}, rotation = 0)));
equation
  connect(genrou.AccPower, ieeest1.PSS_AUX2[2]) annotation(Line(points = {{75, -3.9}, {86.914, -3.9}, {86.914, -38.5559}, {-73.3541, -38.5559}, {-73.3541, 33.9814}, {-69.76, 33.9814}, {-69.76, 33.9814}}, color = {0, 0, 127}));
  connect(genrou.SLIP, ieeest1.PSS_AUX2[1]) annotation(Line(points = {{75, -12}, {80.0524, -12}, {80.0524, -38.5559}, {-73.3541, -38.5559}, {-73.3541, 33.9814}, {-70.0867, 33.9814}, {-70.0867, 33.9814}}, color = {0, 0, 127}));
  connect(ieeest1.VI, genrou.VI) annotation(Line(points = {{-70, 42}, {-74.4977, 42}, {-74.4977, 56.6902}, {89.528, 56.6902}, {89.528, 0}, {75.1512, 0}, {75.1512, 0}}, color = {0, 0, 127}));
  connect(ieeest1.PSS_AUX2, ieeest1.PSS_AUX) annotation(Line(points = {{-70, 34}, {-73.3541, 34}, {-73.3541, 39.6995}, {-69.76, 39.6995}, {-69.76, 39.6995}}, color = {0, 0, 127}));
  connect(ieeest1.VI2, ieeest1.VI) annotation(Line(points = {{-70, 36}, {-74.8245, 36}, {-74.8245, 41.6599}, {-70.2501, 41.6599}, {-70.2501, 41.6599}}, color = {0, 0, 127}));
  connect(esdc2a.VI, genrou.VI) annotation(Line(points = {{15, 23}, {89.4602, 23}, {89.4602, 0}, {76.0925, 0}, {76.0925, 0}}, color = {0, 0, 127}));
  connect(genrou.MBASE, ieeeg1.MBASE) annotation(Line(points = {{75, -8.1}, {84.8329, -8.1}, {84.8329, -42.1594}, {-23.6504, -42.1594}, {-23.6504, -26.7352}, {-12.0823, -26.7352}, {-12.0823, -26.7352}}, color = {0, 0, 127}));
  connect(genrou.VI, ieeeg1.VI) annotation(Line(points = {{75, 0}, {89.9743, 0}, {89.9743, -46.0154}, {-30.3342, -46.0154}, {-30.3342, -21.3368}, {-12.8535, -21.3368}, {-12.8535, -21.3368}}, color = {0, 0, 127}));
  connect(dGREF, ieeeg1.dGREF) annotation(Line(points = {{-53, -10}, {-23.6504, -10}, {-23.6504, -8.74036}, {-12.0823, -8.74036}, {-12.0823, -8.74036}}));
  connect(dVREF, esdc2a.dVREF) annotation(Line(points = {{-52, 8}, {-40.617, 8}, {-40.617, 5.14139}, {-15.4242, 5.14139}, {-15.4242, 5.14139}}));
  connect(genrou.EX_AUX, esdc2a.EX_AUX) annotation(Line(points = {{44.7, 12}, {16.3042, 12}, {16.3042, 12.0136}, {16.3042, 12.0136}}, color = {0, 0, 127}));
  connect(genrou.p, bus0) annotation(Line(points = {{60, -15}, {59.8229, -15}, {60, -23.1691}, {60, -28}}));
  connect(ieeest1.VOTHSG, esdc2a.VOTHSG) annotation(Line(points = {{-50, 34}, {-24.395, 34}, {-24.395, 8.58115}, {-15.4461, 8.58115}, {-15.4461, 8.58115}}, color = {0, 0, 127}));
  connect(esdc2a.EFD, genrou.EFD) annotation(Line(points = {{15, 5}, {41.8025, 5}, {41.8025, 4.0454}, {44.009, 4.0454}, {44.009, 4.0454}}, color = {0, 0, 127}));
  connect(TRIP, genrou.TRIP) annotation(Line(points = {{40, 20}, {60.3132, 20}, {60.3132, 15}, {60, 15}}));
  connect(const.y, esdc2a.VUEL) annotation(Line(points = {{-85.5, 20}, {-23.2611, 20}, {-23.2611, 16.8757}, {-16.6477, 16.8757}, {-16.6477, 16.8757}}, color = {0, 0, 127}));
  connect(esdc2a.VOEL, const.y) annotation(Line(points = {{-15, 12.8}, {-23.2611, 12.8}, {-23.2611, 20.0684}, {-54.9601, 20}, {-85.5, 20}}, color = {0, 0, 127}));
  connect(genrou.XADIFD, esdc2a.XADIFD) annotation(Line(points = {{75, 7.5}, {80.2951, 7.5}, {80.2951, 45.6027}, {-19.4915, 45.6027}, {-19.4915, 21.5755}, {-16.4268, 21.5755}, {-16.4268, 21.5755}}, color = {0, 0, 127}));
  connect(genrou.SLIP, ieeeg1.SLIP) annotation(Line(points = {{75, -12}, {80.0456, -12}, {80.0456, -38.5107}, {-15.8409, -38.5107}, {-15.9635, -32.6112}, {-11.4025, -32.6112}, {-11.4025, -32.6112}}, color = {0, 0, 127}));
  connect(genrou.PMECH0, ieeeg1.PMECH0) annotation(Line(points = {{45, -12}, {20.0684, -12}, {20.0684, -12.3147}, {20.0684, -12.3147}}, color = {0, 0, 127}));
  connect(ieeeg1.PMECH, genrou.PMECH) annotation(Line(points = {{19, -9}, {44.4698, -9}, {44.4698, -9.35006}, {44.4698, -9.35006}}, color = {0, 0, 127}));
  connect(genrou.EFD0, esdc2a.EFD0) annotation(Line(points = {{44.7, 0}, {21.2087, 0}, {21.2087, 2.05245}, {15.9635, 2.05245}, {15.9635, 2.05245}, {15.9635, 2.05245}}, color = {0, 0, 127}));
  connect(genrou.ETERM0, esdc2a.ETERM0) annotation(Line(points = {{44.7, 7.5}, {15.9635, 7.5}, {15.9635, 8.20981}, {15.9635, 8.20981}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {1.21951, -1.08401}, extent = {{-99.3225, 99.187}, {96.6125, -97.019}}), Text(origin = {64.2062, 72.7566}, extent = {{-9.35, 9.08}, {24.7972, -23.1721}}, textString = "dVREF"), Text(origin = {-77.5061, -54.468}, extent = {{-10.57, 8.94}, {20.8681, -17.6121}}, textString = "dGREF"), Text(origin = {-77.2845, 61.9043}, extent = {{-17.21, 12.33}, {19.649, -2.5739}}, textString = "TRIP"), Text(origin = {-4.60478, 8.80816}, extent = {{-89.97, 12.87}, {101.081, -27.7751}}, textString = "GENROU_ESDC2A_IEEEST_IEEEG1")}));
end GENROU_ESDC2A_IEEEST_IEEEG1;

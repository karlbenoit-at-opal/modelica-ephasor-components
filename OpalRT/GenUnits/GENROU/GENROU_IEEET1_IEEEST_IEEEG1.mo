within OpalRT.GenUnits.GENROU;
class GENROU_IEEET1_IEEEST_IEEEG1
  parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 1000 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 100 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 0.95 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -2 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1200 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 60 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 10.2 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.5 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 1.02 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 8.2 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 3 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.5 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.5231 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.361 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.41 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.2 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.5 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.6 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // IEEET1 Parameters
  // This Model is located at system bus IBUS
  parameter Real TR_ex = 0.025 "(sec)";
  parameter Real KA_ex = 98;
  parameter Real TA_ex = 0.2 "(sec)";
  parameter Real VRMAX_ex = 9 "or zero";
  parameter Real VRMIN_ex = -5;
  parameter Real KE_ex = 0.5 "or zero";
  parameter Real TE_ex = 0.35 "(>0) (sec)";
  parameter Real KF_ex = 0.03;
  parameter Real TF_ex = 0.4 "(>0) (sec)";
  parameter Real Switch_ex = 0;
  parameter Real E1_ex = 4;
  parameter Real SE_E1_ex = 0.4;
  parameter Real E2_ex = 5;
  parameter Real SE_E2_ex = 0.5;
  //
  // IEEEG1 Parameters
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
  //
  // IEEEST Parameters
  parameter Real A1_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A2_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A3_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A4_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A5_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A6_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
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
  //
  //
  //****************************
  //
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {55, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-16, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {65, -45}, extent = {{-7, -7}, {7, 7}}, rotation = 0), iconTransformation(origin = {66, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.IEEEST ieeest1(A1 = A1_pss, A2 = A2_pss, A3 = A3_pss, A4 = A4_pss, A5 = A5_pss, A6 = A6_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, KS = KS_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss) annotation(Placement(visible = true, transformation(origin = {-32, 2}, extent = {{-15, -15}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEET1 ieeet11(TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex, Switch = Switch_ex) annotation(Placement(visible = true, transformation(origin = {12, -2}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG1 ieeeg11(K = K_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, Uo = Uo_tg, Uc = Uc_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, T4 = T4_tg, K1 = K1_tg, K2 = K2_tg, T5 = T5_tg, K3 = K3_tg, K4 = K4_tg, T6 = T6_tg, K5 = K5_tg, K6 = K6_tg, T7 = T7_tg, K7 = K7_tg, K8 = K8_tg) annotation(Placement(visible = true, transformation(origin = {12, -36}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-30, -38}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {-42, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-30, -22}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
equation
  connect(dVREF, ieeet11.dVREF) annotation(Line(points = {{-30, -22}, {-15.3424, -22}, {-15.3424, -11.2024}, {-3.1659, -11.2024}, {-3.1659, -11.2024}}));
  connect(dGREF, ieeeg11.dGREF) annotation(Line(points = {{-30, -38}, {-12.9071, -38}, {-12.9071, -24.3531}, {-3, -24.3531}, {-3, -24}}));
  connect(genrou1.VI, ieeest1.VI2) annotation(Line(points = {{70, -15}, {79.6346, -15}, {79.6346, -63.5615}, {-56.7427, -63.5615}, {-56.7427, -5.60121}, {-46.5144, -5.60121}, {-46.5144, -5.60121}}, color = {0, 0, 127}));
  connect(genrou1.AccPower, ieeest1.PSS_AUX2[2]) annotation(Line(points = {{70, -18.9}, {78.6605, -18.9}, {78.6605, -61.3698}, {-52.1156, -61.3698}, {-52.1156, -8.28005}, {-46.7579, -8.28005}, {-46.7579, -8.28005}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, ieeest1.PSS_AUX2[1]) annotation(Line(points = {{70, -27}, {75.7381, -27}, {75.7381, -57.9603}, {-52.1156, -57.9603}, {-52.1156, -8.52358}, {-47.245, -8.52358}, {-47.245, -8.52358}}, color = {0, 0, 127}));
  connect(ieeest1.VI2, ieeest1.VI) annotation(Line(points = {{-47, -5.5}, {-56.7427, -5.5}, {-56.7427, 2.19178}, {-46.7579, 2.19178}, {-46.7579, 2.19178}}, color = {0, 0, 127}));
  connect(genrou1.VI, ieeeg11.VI) annotation(Line(points = {{70, -15}, {79.6346, -15}, {79.6346, -63.5615}, {-12.1765, -63.5615}, {-12.1765, -36.0426}, {-2.19178, -36.0426}, {-2.19178, -36.0426}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, ieeeg11.MBASE) annotation(Line(points = {{70, -23.1}, {77.6863, -23.1}, {77.6863, -60.3956}, {-10.2283, -60.3956}, {-10.2283, -42.3744}, {-3.1659, -42.3744}, {-3.1659, -42.3744}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, ieeeg11.PMECH0) annotation(Line(points = {{40, -27}, {28.006, -27}, {28.006, -27.2755}, {28.006, -27.2755}}, color = {0, 0, 127}));
  connect(ieeeg11.SLIP, genrou1.SLIP) annotation(Line(points = {{-3, -48}, {-8, -48}, {-8, -58}, {76, -58}, {76, -27}, {70, -27}}, color = {0, 0, 127}));
  connect(ieeeg11.PMECH, genrou1.PMECH) annotation(Line(points = {{27, -24}, {32.5, -24}, {32.5, -24}, {40, -24}}, color = {0, 0, 127}));
  connect(ieeet11.VI, genrou1.VI) annotation(Line(points = {{27, 7}, {77.9299, 7}, {77.9299, -14.8554}, {70.3804, -14.8554}, {70.3804, -14.8554}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{60, 40}, {59.4533, 40}, {59.4533, 21.8679}, {54.8975, 21.8679}, {54.8975, 0}, {55, 0}}));
  connect(ieeet11.EFD0, genrou1.EFD0) annotation(Line(points = {{27, -14}, {34, -14}, {34, -15}, {39.7, -15}}, color = {0, 0, 127}));
  connect(ieeet11.EFD, genrou1.EFD) annotation(Line(points = {{27, -11}, {33.5, -11}, {33.5, -10.8}, {40, -10.8}}, color = {0, 0, 127}));
  connect(ieeet11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{27, -8}, {34, -8}, {34, -7.5}, {39.7, -7.5}}, color = {0, 0, 127}));
  connect(ieeet11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{27, -4.1}, {33.5, -4.1}, {33.5, -3}, {39.7, -3}}, color = {0, 0, 127}));
  connect(ieeest1.VOTHSG, ieeet11.VOTHSG) annotation(Line(points = {{-22, -8}, {-10, -8}, {-10, -7.4}, {-3, -7.4}}, color = {0, 0, 127}));
  connect(ieeet11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-3, 5.5}, {-10, 5.5}, {-10, 18}, {76, 18}, {76, -7.5}, {70, -7.5}}, color = {0, 0, 127}));
  connect(const.y, ieeet11.VUEL) annotation(Line(points = {{-10.5, 0}, {-6, 0}, {-6, 1}, {-3, 1}}, color = {0, 0, 127}));
  connect(ieeet11.VOEL, ieeet11.VUEL) annotation(Line(points = {{-3, -3.2}, {-6, -3.2}, {-6, 1}, {-3, 1}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{65, -45}, {55, -45}, {55, -30}}, color = {0, 0, 0}));
  connect(ieeest1.PSS_AUX2, ieeest1.PSS_AUX) annotation(Line(points = {{-46.75, -8}, {-52, -8}, {-52, -0.5}, {-47, -0.5}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-51.82, 7.29}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROU_IEEET1_IEEEST_IEEEG1"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN")}));
end GENROU_IEEET1_IEEEST_IEEEG1;

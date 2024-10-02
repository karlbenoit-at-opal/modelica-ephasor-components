within OpalRT.GenUnits.GENROU;
class GENROU_SCRX_IEEEG1
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
  // SCRX Parameters
  parameter Real TA_TB_ex = 4 "TA/TB" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TB_ex = 1 "(>0) (sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real K_ex = 100 annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TE_ex = 0.5 "(sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMIN_ex = -1.2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMAX_ex = 2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real CSWITCH_ex = 1 "0 for bus fed, 1 for solid fed" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real rc_rfd_ex = 3 "rc/rfd, 0 with negative field current capability (EX=EFD)" annotation(Dialog(tab = "SCRX Parameters"));
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
  //
  //****************************
  //
  //
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG1 ieeeg11(IBUS = 100, ID = M_ID, K = K_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, Uo = Uo_tg, Uc = Uc_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, T4 = T4_tg, K1 = K1_tg, K2 = K2_tg, T5 = T5_tg, K3 = K3_tg, K4 = K4_tg, T6 = T6_tg, K5 = K5_tg, K6 = K6_tg, T7 = T7_tg, K7 = K7_tg, K8 = K8_tg) annotation(Placement(visible = true, transformation(origin = {10, -34}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.SCRX scrx1(IBUS = IBUS, ID = M_ID, TA_TB = TA_TB_ex, TB = TB_ex, K = K_ex, TE = TE_ex, EMIN = EMIN_ex, EMAX = EMAX_ex, CSWITCH = CSWITCH_ex, rc_rfd = rc_rfd_ex) annotation(Placement(visible = true, transformation(origin = {11, -1}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {55, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-22, 2}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-43, -17}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-43, -33}, extent = {{-7, -7}, {7, 7}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ieeeg11.PMECH0, genrou1.PMECH0) annotation(Line(points = {{25, -25}, {29.7302, -25}, {29.7302, -27.2755}, {39.5494, -27.2755}, {39.5494, -27.2755}}, color = {0, 0, 127}));
  connect(scrx1.VI, genrou1.VI) annotation(Line(points = {{26, 8}, {83.4629, 8}, {83.4629, -15.547}, {70.9162, -15.547}, {70.9162, -15.547}}, color = {0, 0, 127}));
  connect(ieeeg11.MBASE, genrou1.MBASE) annotation(Line(points = {{-5, -40}, {-15.2743, -40}, {-15.2743, -69.2797}, {75.8258, -69.2797}, {75.8258, -23.7296}, {71.1889, -23.7296}, {71.1889, -23.7296}}, color = {0, 0, 127}));
  connect(ieeeg11.VI, genrou1.VI) annotation(Line(points = {{-5, -34}, {-17.729, -34}, {-17.729, -71.1889}, {78.2806, -71.1889}, {78.2806, -15.547}, {71.4617, -15.547}, {71.4617, -15.547}}, color = {0, 0, 127}));
  connect(dGREF, ieeeg11.dGREF) annotation(Line(points = {{-43, -33}, {-23.1841, -33}, {-23.1841, -22.3659}, {-4.90958, -22.3659}, {-4.90958, -22.3659}}));
  connect(dVREF, scrx1.dVREF) annotation(Line(points = {{-43, -17}, {-19.3656, -17}, {-19.3656, -10.6374}, {-3.81856, -10.6374}, {-3.81856, -10.6374}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{60, 40}, {59.4533, 40}, {59.4533, 21.8679}, {54.8975, 21.8679}, {54.8975, 0}, {55, 0}}));
  connect(scrx1.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{26, -3.1}, {34, -3.1}, {34, -3}, {39.7, -3}}, color = {0, 0, 127}));
  connect(scrx1.ETERM0, genrou1.ETERM0) annotation(Line(points = {{26, -7}, {34, -7}, {34, -7.5}, {39.7, -7.5}}, color = {0, 0, 127}));
  connect(scrx1.EFD, genrou1.EFD) annotation(Line(points = {{26, -10}, {34, -10}, {34, -10.8}, {40, -10.8}}, color = {0, 0, 127}));
  connect(scrx1.EFD0, genrou1.EFD0) annotation(Line(points = {{26, -13}, {34, -13}, {34, -15}, {39.7, -15}}, color = {0, 0, 127}));
  connect(ieeeg11.PMECH, genrou1.PMECH) annotation(Line(points = {{25, -22}, {30, -22}, {30, -24}, {40, -24}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, ieeeg11.SLIP) annotation(Line(points = {{70, -27}, {74, -27}, {74, -64}, {-12, -64}, {-12, -46}, {-5, -46}}, color = {0, 0, 127}));
  connect(genrou1.XADIFD, scrx1.XADIFD) annotation(Line(points = {{70, -7.5}, {74, -7.5}, {74, -8}, {78, -8}, {78, 20}, {-10, 20}, {-10, 6.5}, {-4, 6.5}}, color = {0, 0, 127}));
  connect(const.y, scrx1.VUEL) annotation(Line(points = {{-16.5, 2}, {-4, 2}}, color = {0, 0, 127}));
  connect(scrx1.VOEL, scrx1.VUEL) annotation(Line(points = {{-4, -2.2}, {-8, -2.2}, {-8, -2}, {-10, -2}, {-10, 2}, {-4, 2}}, color = {0, 0, 127}));
  connect(scrx1.VOTHSG, scrx1.VUEL) annotation(Line(points = {{-4, -6.4}, {-10, -6.4}, {-10, 2}, {-4, 2}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{100, -60}, {55, -60}, {55, -30}}, color = {0, 0, 0}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-51.82, 7.29}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROU_SCRX_IEEEG1"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN")}));
end GENROU_SCRX_IEEEG1;

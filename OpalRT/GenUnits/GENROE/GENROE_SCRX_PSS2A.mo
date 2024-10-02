within OpalRT.GenUnits.GENROE;
class GENROE_SCRX_PSS2A
  parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
  // GENROE Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROE Parameters"));
  // SCRX Parameters
  parameter Real TA_TB_ex = 4 "TA/TB" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TB_ex = 1 "(>0) (sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real K_ex = 100 annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TE_ex = 0.5 "(sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMIN_ex = -1.2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMAX_ex = 2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real CSWITCH_ex = 1 "0 for bus fed, 1 for solid fed" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real rc_rfd_ex = 3 "rc/rfd, 0 with negative field current capability (EX=EFD)" annotation(Dialog(tab = "SCRX Parameters"));
  // PSS2A Parameters
  parameter Real TW1_pss = 0.1 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW2_pss = 1 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T6_pss = 1 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW3_pss = 1.5 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW4_pss = 0.2 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T7_pss = 0.2 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS2_pss = 0.1 "T7/(2*H)" annotation(Dialog(tab = "PSS2A Parameters"));
  //T7/(2*H);
  parameter Real KS3_pss = 0.1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T8_pss = 0.05 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T9_pss = 0.01 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS1_pss = 0.15 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T1_pss = 0.1 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T2_pss = 0.1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T3_pss = 0.01 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T4_pss = 1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMAX_pss = 1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMIN_pss = -1 annotation(Dialog(tab = "PSS2A Parameters"));
  // PSS2A ICONs
  parameter Real M0_pss = 2 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M1_pss = 2 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M2_pss = 2 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M3_pss = 2 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M4_pss = 1 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M5_pss = 2 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  //
  //
  //****************************
  //
  //
  OpalRT.Electrical.Control.Stabilizer.PSS2A pss2a1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-60, 4}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus1 annotation(Placement(visible = true, transformation(origin = {80, -12}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.SCRX scrx1(IBUS = IBUS, ID = M_ID, TA_TB = TA_TB_ex, TB = TB_ex, K = K_ex, TE = TE_ex, EMIN = EMIN_ex, EMAX = EMAX_ex, CSWITCH = CSWITCH_ex, rc_rfd = rc_rfd_ex) annotation(Placement(visible = true, transformation(origin = {-10, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROE genroe1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 0) annotation(Placement(visible = true, transformation(origin = {-66, 34}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {20, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(TRIP, genroe1.TRIP) annotation(Line(points = {{20, 40}, {20.0802, 40}, {20.0802, 18}, {20, 18}}));
  connect(genroe1.AccPower, pss2a1.PSS_AUX2[2]) annotation(Line(points = {{30, 5.4}, {42.5403, 5.4}, {42.5403, -20.7677}, {-80.2236, -20.7677}, {-80.2236, -2.00978}, {-75.3666, -2.00978}, {-75.3666, -2.00978}}, color = {0, 0, 127}));
  connect(genroe1.SLIP, pss2a1.PSS_AUX2[1]) annotation(Line(points = {{30, -4.44089e-16}, {42.5403, -4.44089e-16}, {42.5403, -20.7677}, {-80.2236, -20.7677}, {-80.2236, -2.00978}, {-75.5341, -2.00978}, {-75.5341, -2.00978}}, color = {0, 0, 127}));
  connect(const1.y, scrx1.VUEL) annotation(Line(points = {{-59.4, 34}, {-38, 34}, {-38, 18}, {-20, 18}}, color = {0, 0, 127}));
  connect(scrx1.dVREF, dVREF) annotation(Line(points = {{-20, 10}, {-22.275, 10}, {-22.275, 2.51222}, {-22.275, 2.51222}}, color = {0, 0, 127}));
  connect(genroe1.VI, pss2a1.VI2) annotation(Line(points = {{30, 8}, {40.3295, 8}, {40.3295, -15.3815}, {-78.4081, -15.3815}, {-78.4081, -0.187579}, {-75.5944, -0.187579}, {-75.5944, -0.375158}, {-75.5944, -0.375158}}, color = {0, 0, 127}));
  connect(pss2a1.VI, pss2a1.VI2) annotation(Line(points = {{-75, 6}, {-78.2078, 6}, {-78.2078, -0.299074}, {-75.8152, -0.299074}, {-75.8152, -0.299074}}, color = {0, 0, 127}));
  connect(genroe1.VI, scrx1.VI) annotation(Line(points = {{30, 8}, {40.1955, 8}, {40.1955, 21.9401}, {0.334963, 21.9401}, {0.334963, 21.9401}}, color = {0, 0, 127}));
  connect(genroe1.p, bus1) annotation(Line(points = {{20, -2}, {20, -12}, {80, -12}}));
  connect(scrx1.EFD0, genroe1.EFD0) annotation(Line(points = {{0, 8}, {9.8, 8}}, color = {0, 0, 127}));
  connect(scrx1.EFD, genroe1.EFD) annotation(Line(points = {{0, 10}, {4, 10}, {4, 10.8}, {10, 10.8}}, color = {0, 0, 127}));
  connect(scrx1.ETERM0, genroe1.ETERM0) annotation(Line(points = {{0, 12}, {4, 12}, {4, 13}, {9.8, 13}}, color = {0, 0, 127}));
  connect(scrx1.EX_AUX, genroe1.EX_AUX) annotation(Line(points = {{0, 14.6}, {4, 14.6}, {4, 16}, {9.8, 16}}, color = {0, 0, 127}));
  connect(scrx1.XADIFD, genroe1.XADIFD) annotation(Line(points = {{-20, 21}, {-26, 21}, {-26, 32}, {38, 32}, {38, 13}, {30, 13}}, color = {0, 0, 127}));
  connect(scrx1.VOEL, scrx1.VUEL) annotation(Line(points = {{-20, 15.2}, {-24, 15.2}, {-24, 18}, {-20, 18}}, color = {0, 0, 127}));
  connect(pss2a1.VOTHSG, scrx1.VOTHSG) annotation(Line(points = {{-45, -2}, {-38, -2}, {-38, 12.4}, {-20, 12.4}}, color = {0, 0, 127}));
  connect(pss2a1.PSS_AUX, pss2a1.PSS_AUX2) annotation(Line(points = {{-75, 4}, {-80, 4}, {-80, -2}, {-74.7, -2}}, color = {0, 0, 127}));
  connect(genroe1.PMECH0, genroe1.PMECH) annotation(Line(points = {{10, 0}, {6, 0}, {6, 2}, {10, 2}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-51.82, 7.29}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROE_SCRX_PSS2A_GAST"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN"), Text(origin = {-68.1086, 61.7315}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "dGREF"), Text(origin = {-73.8448, -60.1801}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "TRIP")}));
end GENROE_SCRX_PSS2A;

within OpalRT.GenUnits.GENROU;
class GENROU_EXST1_IEESGO
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
  // EXST1 Parameters
  parameter String EX_ID = M_ID "Machine Identifier" annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TR_ex = 0.02 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VIMAX_ex = 0.2 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VIMIN_ex = 0 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TC_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TB_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KA_ex = 500 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TA_ex = 0.05 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMAX_ex = 8 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMIN_ex = -3 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KC_ex = 0.2 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KF_ex = 0.1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TF_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  // IEESGO Parameters
  parameter Real T1_tg = 1 "Controller Lag" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real T2_tg = 1 "Controller Lead Compensation" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real T3_tg = 1 "Governor Lag (> 0)" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real T4_tg = 1 "Delay Due To Steam Inlet Volumes" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real T5_tg = 1 "Reheater Delay" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real T6_tg = 1 "Turbine pipe hood Delay" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real K1_tg = 0.5 "1/Per Unit Regulation" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real K2_tg = 0.5 "Fraction" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real K3_tg = 0.5 "fraction" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real PMAX_tg = 1 "Upper Power Limit" annotation(Dialog(tab = "IEESGO Parameters"));
  parameter Real PMIN_tg = -1 "Lower Power Limit" annotation(Dialog(tab = "IEESGO Parameters"));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {55, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXST1 exst11(ID = EX_ID, TR = TR_ex, VIMIN = VIMIN_ex, VIMAX = VIMAX_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex) annotation(Placement(visible = true, transformation(origin = {21, 2}, extent = {{-20, -20}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEESGO ieesgo1(T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, T4 = T4_tg, T5 = T5_tg, T6 = T6_tg, K1 = K1_tg, K2 = K2_tg, K3 = K3_tg, PMAX = PMAX_tg, PMIN = PMIN_tg) annotation(Placement(visible = true, transformation(origin = {16, -36}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {56, -48}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-32, -12}, extent = {{-4.5, -4.5}, {4.5, 4.5}}, rotation = 0), iconTransformation(origin = {-36, -18}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-30, -24}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(dGREF, ieesgo1.dGREF) annotation(Line(points = {{-30, -24}, {-17.1835, -24}, {-17.1835, -24.5479}, {1.63653, -24.5479}, {1.63653, -24.5479}}));
  connect(dVREF, exst11.dVREF) annotation(Line(points = {{-32, -12}, {-17.1835, -12}, {-17.1835, -12.5467}, {0.818264, -12.5467}, {0.818264, -12.5467}}));
  connect(bus0, genrou1.p) annotation(Line(points = {{56, -48}, {54, -48}, {54, -30}, {55, -30}}));
  connect(ieesgo1.MBASE, genrou1.MBASE) annotation(Line(points = {{1, -42}, {-9.54641, -42}, {-9.54641, -70.3707}, {75.553, -70.3707}, {75.553, -23.1841}, {70.9162, -23.1841}, {70.9162, -23.1841}}, color = {0, 0, 127}));
  connect(ieesgo1.VI, genrou1.VI) annotation(Line(points = {{1, -36}, {-12.8195, -36}, {-12.8195, -76.644}, {80.4626, -76.644}, {80.4626, -15.2743}, {71.1889, -15.2743}, {71.1889, -15.2743}, {71.1889, -15.2743}}, color = {0, 0, 127}));
  connect(ieesgo1.PMECH0, genrou1.PMECH0) annotation(Line(points = {{31, -27}, {39.5494, -27}, {39.5494, -27.0027}, {39.5494, -27.0027}}, color = {0, 0, 127}));
  connect(exst11.VI, genrou1.VI) annotation(Line(points = {{31, 6}, {80.4626, 6}, {80.4626, -15.2743}, {71.1889, -15.2743}, {71.1889, -15.2743}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{60, 40}, {60.1367, 40}, {60.1367, 21.1845}, {55.1253, 21.1845}, {55.1253, 0}, {55, 0}}));
  connect(exst11.EFD0, genrou1.EFD0) annotation(Line(points = {{31, -15}, {35.5, -15}, {35.5, -15}, {39.7, -15}}, color = {0, 0, 127}));
  connect(exst11.EFD, genrou1.EFD) annotation(Line(points = {{31, -12}, {36, -12}, {36, -10.8}, {40, -10.8}}, color = {0, 0, 127}));
  connect(exst11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{31, -9}, {35.5, -9}, {35.5, -7.5}, {39.7, -7.5}}, color = {0, 0, 127}));
  connect(exst11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{31, -5.1}, {35.5, -5.1}, {35.5, -3}, {39.7, -3}}, color = {0, 0, 127}));
  connect(exst11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{1, 4.5}, {-4, 4.5}, {-4, 16}, {76, 16}, {76, -7.5}, {70, -7.5}}, color = {0, 0, 127}));
  connect(genrou1.PMECH, ieesgo1.PMECH) annotation(Line(points = {{40, -24}, {34, -24}, {34, -24}, {31, -24}}, color = {0, 0, 127}));
  connect(const.y, exst11.VUEL) annotation(Line(points = {{-14.5, 0}, {1, 0}}, color = {0, 0, 127}));
  connect(exst11.VOEL, exst11.VUEL) annotation(Line(points = {{1, -4.2}, {-6, -4.2}, {-6, 0}, {1, 0}}, color = {0, 0, 127}));
  connect(exst11.VOTHSG, exst11.VUEL) annotation(Line(points = {{1, -8.4}, {-6, -8.4}, {-6, 0}, {1, 0}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, ieesgo1.SLIP) annotation(Line(points = {{70, -27}, {72, -27}, {72, -64}, {-4, -64}, {-4, -48}, {1, -48}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-51.82, 7.29}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROU_EXST1_PSS2A_IEESGO"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN"), Text(origin = {-68.1086, 61.7315}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "dGREF"), Text(origin = {-73.8448, -60.1801}, extent = {{-28.7, 7.52}, {28.7, -7.52}}, textString = "TRIP")}));
end GENROU_EXST1_IEESGO;

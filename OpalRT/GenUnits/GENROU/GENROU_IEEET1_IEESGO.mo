within OpalRT.GenUnits.GENROU;
class GENROU_IEEET1_IEESGO
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
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
  OpalRT.Electrical.Control.TurbineGovernor.IEESGO ieesgo1(T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, T4 = T4_tg, T5 = T5_tg, T6 = T6_tg, K1 = K1_tg, K2 = K2_tg, K3 = K3_tg, PMAX = PMAX_tg, PMIN = PMIN_tg) annotation(Placement(visible = true, transformation(origin = {-20, -20}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEET1 ieeet11(ID = M_ID, TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {-20, 12}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {27, -33}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {20, 26}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-56, 16}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-72, -8}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-65, 5}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-80, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ieesgo1.MBASE, genrou1.MBASE) annotation(Line(points = {{-35, -26}, {-42.3228, -26}, {-42.3228, -47.7588}, {46.2056, -47.7588}, {46.2056, -8.34808}, {36.1103, -8.34808}, {36.1103, -8.34808}}, color = {0, 0, 127}));
  connect(ieesgo1.VI, genrou1.VI) annotation(Line(points = {{-35, -20}, {-45.6232, -20}, {-45.6232, -50.6709}, {48.9236, -50.6709}, {48.9236, -0.194141}, {35.722, -0.194141}, {35.722, -0.582424}, {35.722, -0.582424}}, color = {0, 0, 127}));
  connect(ieeet11.VI, genrou1.VI) annotation(Line(points = {{-5, 21}, {48.7295, 21}, {48.7295, -0.194141}, {35.9161, -0.194141}, {35.9161, -0.194141}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, ieesgo1.PMECH0) annotation(Line(points = {{5, -12}, {1.16485, -12}, {1.16485, -11.4543}, {-4.07697, -11.4543}, {-4.07697, -11.4543}}, color = {0, 0, 127}));
  connect(dGREF, ieesgo1.dGREF) annotation(Line(points = {{-72, -8}, {-49.3119, -8}, {-49.3119, -7.95979}, {-35.5279, -7.95979}, {-35.5279, -7.95979}}));
  connect(dVREF, ieeet11.dVREF) annotation(Line(points = {{-65, 5}, {-54.3596, 5}, {-54.3596, 2.71798}, {-34.5572, 2.71798}, {-34.5572, 2.71798}}));
  connect(genrou1.TRIP, TRIP) annotation(Line(points = {{20, 15}, {20.2733, 15}, {20.2733, 26}, {20, 26}}, color = {0, 0, 127}));
  connect(ieeet11.EFD0, genrou1.EFD0) annotation(Line(points = {{-5, 0}, {4.7, 0}}, color = {0, 0, 127}));
  connect(ieeet11.EFD, genrou1.EFD) annotation(Line(points = {{-5, 3}, {-0.5, 3}, {-0.5, 4.2}, {5, 4.2}}, color = {0, 0, 127}));
  connect(ieeet11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{-5, 6}, {0, 6}, {0, 7.5}, {4.7, 7.5}}, color = {0, 0, 127}));
  connect(ieeet11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{-5, 9.9}, {-0.5, 9.9}, {-0.5, 12}, {4.7, 12}}, color = {0, 0, 127}));
  connect(ieeet11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-35, 19.5}, {-42, 19.5}, {-42, 34}, {44, 34}, {44, 7.5}, {35, 7.5}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{27, -33}, {20, -33}, {20, -15}}, color = {0, 0, 0}));
  connect(const.y, ieeet11.VUEL) annotation(Line(points = {{-50.5, 16}, {-42, 16}, {-42, 15}, {-35, 15}}, color = {0, 0, 127}));
  connect(ieeet11.VOEL, ieeet11.VUEL) annotation(Line(points = {{-35, 10.8}, {-42, 10.8}, {-42, 15}, {-35, 15}}, color = {0, 0, 127}));
  connect(ieeet11.VOTHSG, ieeet11.VUEL) annotation(Line(points = {{-35, 6.6}, {-42, 6.6}, {-42, 15}, {-35, 15}}, color = {0, 0, 127}));
  connect(ieesgo1.PMECH, genrou1.PMECH) annotation(Line(points = {{-5, -8}, {-2, -8}, {-2, -9}, {5, -9}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, ieesgo1.SLIP) annotation(Line(points = {{35, -12}, {42, -12}, {42, -44}, {-40, -44}, {-40, -32}, {-35, -32}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {-1.36674, 26.4237}, extent = {{-88.3827, 36.4465}, {94.3052, -90.4328}}), Text(origin = {-31.89, -1.02}, extent = {{-48.06, 27.45}, {116.4, -21.07}}, textString = "GENROU_IEEET1_IEESGO")}));
end GENROU_IEEET1_IEESGO;

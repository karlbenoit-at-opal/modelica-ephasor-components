within OpalRT.GenUnits.GENROU;
class GENROU_IEEET2_IEESGO
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
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROU"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU"));
  // IEEET2 Parameters
  parameter Real TR_ex = 0.025 "(sec)" annotation(Dialog(tab = "IEEET2"));
  parameter Real KA_ex = 98 annotation(Dialog(tab = "IEEET2"));
  parameter Real TA_ex = 0.2 "(sec)" annotation(Dialog(tab = "IEEET2"));
  parameter Real VRMAX_ex = 9 "or zero" annotation(Dialog(tab = "IEEET2"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "IEEET2"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "IEEET2"));
  parameter Real TE_ex = 0.35 "(>0) (sec)" annotation(Dialog(tab = "IEEET2"));
  parameter Real KF_ex = 0.03 annotation(Dialog(tab = "IEEET2"));
  parameter Real TF1_ex = 0.4 "(>0) (sec)" annotation(Dialog(tab = "IEEET2"));
  parameter Real TF2_ex = 0.4 "(>0) (sec)" annotation(Dialog(tab = "IEEET2"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "IEEET2"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "IEEET2"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "IEEET2"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "IEEET2"));
  // IEESGO Parameters
  parameter Real T1_tg = 1 "Controller Lag" annotation(Dialog(tab = "IEESGO"));
  parameter Real T2_tg = 1 "Controller Lead Compensation" annotation(Dialog(tab = "IEESGO"));
  parameter Real T3_tg = 1 "Governor Lag (> 0)" annotation(Dialog(tab = "IEESGO"));
  parameter Real T4_tg = 1 "Delay Due To Steam Inlet Volumes" annotation(Dialog(tab = "IEESGO"));
  parameter Real T5_tg = 1 "Reheater Delay" annotation(Dialog(tab = "IEESGO"));
  parameter Real T6_tg = 1 "Turbine pipe hood Delay" annotation(Dialog(tab = "IEESGO"));
  parameter Real K1_tg = 0.5 "1/Per Unit Regulation" annotation(Dialog(tab = "IEESGO"));
  parameter Real K2_tg = 0.5 "Fraction" annotation(Dialog(tab = "IEESGO"));
  parameter Real K3_tg = 0.5 "fraction" annotation(Dialog(tab = "IEESGO"));
  parameter Real PMAX_tg = 1 "Upper Power Limit" annotation(Dialog(tab = "IEESGO"));
  parameter Real PMIN_tg = -1 "Lower Power Limit" annotation(Dialog(tab = "IEESGO"));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {20, 60}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0), iconTransformation(origin = {-196, 74}, extent = {{-13.25, -13.25}, {13.25, 13.25}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-26, -26}, {26, 26}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {20, -60}, extent = {{-9.5, -9.5}, {9.5, 9.5}}, rotation = 0), iconTransformation(origin = {101, -71}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-160, -8}, extent = {{-9.5, -9.5}, {9.5, 9.5}}, rotation = 0), iconTransformation(origin = {98, 70}, extent = {{12.5, 12.5}, {-12.5, -12.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-158, 20}, extent = {{-9.25, -9.25}, {9.25, 9.25}}, rotation = 0), iconTransformation(origin = {-196, -72}, extent = {{-13.25, -13.25}, {13.25, 13.25}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEET2 ieeet21(TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, TF2 = TF2_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {-56, 24}, extent = {{-27.5, -27.5}, {27.5, 27.5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = 0) annotation(Placement(visible = true, transformation(origin = {-118, 44}, extent = {{-7.75, -7.75}, {7.75, 7.75}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEESGO ieesgo1(T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, T4 = T4_tg, T5 = T5_tg, T6 = T6_tg, K1 = K1_tg, K2 = K2_tg, K3 = K3_tg, PMAX = PMAX_tg, PMIN = PMIN_tg) annotation(Placement(visible = true, transformation(origin = {-54, -36}, extent = {{-25.625, -25.625}, {25.625, 25.625}}, rotation = 0)));
equation
  connect(ieesgo1.VI, genrou1.VI) annotation(Line(points = {{-79.625, -36}, {-98.2833, -36}, {-98.2833, -87.1245}, {68.6695, -87.1245}, {68.6695, -0.429185}, {47.2103, -0.429185}, {47.2103, -0.429185}}, color = {0, 0, 127}));
  connect(ieesgo1.MBASE, genrou1.MBASE) annotation(Line(points = {{-79.625, -46.25}, {-92.2747, -46.25}, {-92.2747, -80.6867}, {62.6609, -80.6867}, {62.6609, -14.1631}, {48.0687, -14.1631}, {48.0687, -14.1631}}, color = {0, 0, 127}));
  connect(ieeet21.VI, genrou1.VI) annotation(Line(points = {{-28.5, 40.5}, {63.0901, 40.5}, {63.0901, -0.429185}, {48.4979, -0.429185}, {48.4979, -0.429185}}, color = {0, 0, 127}));
  connect(dVREF, ieeet21.dVREF) annotation(Line(points = {{-158, 20}, {-133.476, 20}, {-133.476, 8.15451}, {-81.9742, 8.15451}, {-81.9742, 8.15451}}));
  connect(dGREF, ieesgo1.dGREF) annotation(Line(points = {{-160, -8}, {-94.4206, -8}, {-94.4206, -15.0215}, {-78.97, -15.0215}, {-78.97, -15.0215}}));
  connect(genrou1.SLIP, ieesgo1.SLIP) annotation(Line(points = {{46, -20.8}, {55.8685, -20.8}, {55.8685, -73.7089}, {-90.6103, -73.7089}, {-90.6103, -56.8075}, {-83.0986, -56.5}, {-79.625, -56.5}}, color = {0, 0, 127}));
  connect(ieesgo1.PMECH, genrou1.PMECH) annotation(Line(points = {{-28.375, -15.5}, {-6.57277, -15.5}, {-6.57277, -15.0235}, {-6.57277, -15.0235}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, ieesgo1.PMECH0) annotation(Line(points = {{-6, -20.8}, {-29.108, -20.8}, {-28.375, -21.5962}, {-28.375, -20.625}}, color = {0, 0, 127}));
  connect(constant2.y, ieeet21.VUEL) annotation(Line(points = {{-109.475, 44}, {-101.878, 44}, {-101.878, 29.5775}, {-83.5, 29.5}, {-83.5, 29.5}}, color = {0, 0, 127}));
  connect(constant2.y, ieeet21.VOEL) annotation(Line(points = {{-109.475, 44}, {-101.878, 44}, {-101.878, 22.0657}, {-83.5, 21.8}, {-83.5, 21.8}}, color = {0, 0, 127}));
  connect(constant2.y, ieeet21.VOTHSG) annotation(Line(points = {{-109.475, 44}, {-101.878, 44}, {-101.878, 14.0845}, {-83.5, 14.1}, {-83.5, 14.1}}, color = {0, 0, 127}));
  connect(genrou1.XADIFD, ieeet21.XADIFD) annotation(Line(points = {{46, 13}, {60.0939, 13}, {60.0939, 81.6901}, {-92.9577, 81.6901}, {-92.9577, 38.0282}, {-83.5, 37.75}, {-83.5, 37.75}}, color = {0, 0, 127}));
  connect(genrou1.EFD0, ieeet21.EFD0) annotation(Line(points = {{-6.52, 0}, {-28.169, 0}, {-28.5, 2}, {-28.5, 2}}, color = {0, 0, 127}));
  connect(ieeet21.EFD, genrou1.EFD) annotation(Line(points = {{-28.5, 7.5}, {-7.04225, 7.5}, {-7.04225, 7.28}, {-6, 7.28}}, color = {0, 0, 127}));
  connect(genrou1.ETERM0, ieeet21.ETERM0) annotation(Line(points = {{-6.52, 13}, {-26.7606, 13}, {-28.5, 13}, {-28.5, 13}}, color = {0, 0, 127}));
  connect(genrou1.EX_AUX, ieeet21.EX_AUX) annotation(Line(points = {{-6.52, 20.8}, {-26.2911, 20.8}, {-26.2911, 20.15}, {-28.5, 20.15}}, color = {0, 0, 127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{20, -26}, {20.6573, -26}, {20.6573, -52.5822}, {20.6573, -52.5822}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{20, 60}, {22.0657, 60}, {22.0657, 28.169}, {22.0657, 28.169}}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-75.7603, 24.371}, extent = {{-116.79, 25.76}, {165.508, -49.6901}}, textString = "GENROU_IEEET2_IEESGO"), Text(origin = {83.258, -83.7962}, extent = {{-39.434, 24.942}, {4.12501, 0.119567}}, textString = "PIN"), Text(origin = {-154.782, 67.94}, extent = {{-32.95, 18.39}, {18.1359, -4.59753}}, textString = "TRIP"), Text(origin = {-152.866, -71.1288}, extent = {{-25.16, 14.43}, {28.7358, -16.7287}}, textString = "dVREF"), Text(origin = {52.1046, 72.2783}, extent = {{-25.8, 17.37}, {30.3975, -19.9242}}, textString = "dGREF"), Rectangle(origin = {-49.8062, 0.383124}, extent = {{-147.631, 97.1859}, {147.631, -97.6967}})}));
end GENROU_IEEET2_IEESGO;

within OpalRT.GenUnits.GENSAL;
class GENSAL_DEGOV1
  parameter Real partType = 1;
  // GENSAL Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tdo_s = 0.05 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tqo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  // DEGOV1 Parameters
  parameter Real M_tg = 1 "Feedback signal flag. 0: Throttle feedback, 1: Electrical power feedback" annotation(Dialog(tab = "DEGOV1"));
  parameter Real T1_tg = 0 "(sec)" annotation(Dialog(tab = "DEGOV1"));
  parameter Real T2_tg = 0 "(sec)" annotation(Dialog(tab = "DEGOV1"));
  parameter Real T3_tg = 1 "(sec)" annotation(Dialog(tab = "DEGOV1"));
  parameter Real K_tg = 0.8 annotation(Dialog(tab = "DEGOV1"));
  parameter Real T4_tg = 0.1 "(sec)" annotation(Dialog(tab = "DEGOV1"));
  parameter Real T5_tg = 0.8 "(sec)" annotation(Dialog(tab = "DEGOV1"));
  parameter Real T6_tg = 0.25 "(sec)" annotation(Dialog(tab = "DEGOV1"));
  parameter Real TD_tg = 0.04 "(0<= TD <= 12*DELT)(sec)" annotation(Dialog(tab = "DEGOV1"));
  parameter Real TMAX_tg = 2.0 annotation(Dialog(tab = "DEGOV1"));
  parameter Real TMIN_tg = 0.5 annotation(Dialog(tab = "DEGOV1"));
  parameter Real DROOP_tg = 0.2 annotation(Dialog(tab = "DEGOV1"));
  parameter Real TE_tg = 0.007 annotation(Dialog(tab = "DEGOV1"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 20}, extent = {{-29, -29}, {29, 29}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.DEGOV1 degov11(M = M_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, K = K_tg, T4 = T4_tg, T5 = T5_tg, T6 = T6_tg, TD = TD_tg, TMAX = TMAX_tg, TMIN = TMIN_tg, DROOP = DROOP_tg, TE = TE_tg) annotation(Placement(visible = true, transformation(origin = {-40, -40}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-80, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(gensal1.VI, degov11.VI) annotation(Line(points = {{69, 20}, {88.4319, 20}, {88.4319, -79.9486}, {-79.6915, -79.9486}, {-79.6915, -40.617}, {-65.5527, -40.617}, {-65.5527, -40.617}}, color = {0, 0, 127}));
  connect(gensal1.MBASE, degov11.MBASE) annotation(Line(points = {{69, 4.34}, {82.7763, 4.34}, {82.7763, -75.8355}, {-76.0925, -75.8355}, {-76.0925, -50.1285}, {-66.3239, -50.1285}, {-66.3239, -50.1285}}, color = {0, 0, 127}));
  connect(degov11.PMECH, gensal1.PMECH) annotation(Line(points = {{-15, -20}, {-5.63991, -20}, {-5.63991, 1.73536}, {9.54447, 1.73536}, {9.54447, 1.73536}}, color = {0, 0, 127}));
  connect(gensal1.PMECH0, degov11.PMECH0) annotation(Line(points = {{11, -3.2}, {-0.867679, -3.2}, {-0.867679, -25.1627}, {-12.5813, -25.1627}, {-12.5813, -25.1627}}, color = {0, 0, 127}));
  connect(degov11.dGREF, dGREF) annotation(Line(points = {{-65, -20}, {-80.2326, -20}, {-80.2326, 15.9884}, {-80.2326, 15.9884}}, color = {0, 0, 127}));
  connect(gensal1.SLIP, degov11.SLIP) annotation(Line(points = {{69, -3.2}, {76.582, -3.2}, {76.582, -71.95}, {-72.2588, -71.95}, {-72.2588, -59.9069}, {-67.6268, -59.9069}, {-67.6268, -59.9069}}, color = {0, 0, 127}));
  connect(gensal1.EFD0, gensal1.EFD) annotation(Line(points = {{10.42, 20}, {3.47072, 20}, {3.47072, 27.3319}, {9.97831, 27.3319}, {9.97831, 27.3319}}, color = {0, 0, 127}));
  connect(gensal1.p, bus0) annotation(Line(points = {{40, -9}, {39.9132, -9}, {39.9132, -36.8764}, {39.9132, -36.8764}}));
  connect(gensal1.TRIP, TRIP) annotation(Line(points = {{40, 49}, {39.9132, 49}, {39.9132, 74.1866}, {39.9132, 74.1866}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.22805, 13.1999}, extent = {{-98.103, 60.2981}, {98.103, -60.2981}}), Text(origin = {69.2421, -19.78}, extent = {{-14.77, 8.13}, {14.77, -8.13}}, textString = "PIN"), Text(origin = {-70.107, 40.604}, extent = {{-14.77, 8.13}, {12.602, -8.94301}}, textString = "TRIP"), Text(origin = {-68.3702, -17.445}, extent = {{-14.77, 8.13}, {20.1881, -10.295}}, textString = "dGREF"), Text(origin = {-53.96, 22.07}, extent = {{-14.77, 8.13}, {124.25, -27.1}}, textString = "GENSAL_DEGOV1")}));
end GENSAL_DEGOV1;

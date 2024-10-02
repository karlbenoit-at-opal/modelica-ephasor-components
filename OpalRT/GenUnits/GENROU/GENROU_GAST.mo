within OpalRT.GenUnits.GENROU;
class GENROU_GAST
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
  // GAST Parameters
  parameter Real R_tg = 0.047 "Speed droop" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T1_tg = 0.4 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T2_tg = 0.1 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T3_tg = 3 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real AT_tg = 1.5 "Ambient temperature load limit" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real KT_tg = 2 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMAX_tg = 1.5 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMIN_tg = 0 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real DTURB_tg = 0 annotation(Dialog(tab = "GAST Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {0, 20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.GAST gast1(R = R_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, AT = AT_tg, KT = KT_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, DTURB = DTURB_tg) annotation(Placement(visible = true, transformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-165, 75}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-198, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-60, 80}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-198, 76}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
equation
  connect(genrou1.VI, gast1.VI) annotation(Line(points = {{-50, 40}, {-40.0395, 40},
        {-40.0395, 90.9308}, {-114.88, 90.9308}, {-114.88, 80.079}, {-110.202, 80.079},
        {-110.202, 80.079}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, gast1.MBASE) annotation(Line(points = {{-50, 34.6}, {-38.3556,
        34.6}, {-38.3556, 92.6148}, {-116.377, 92.6148}, {-116.377, 75.7757}, {-110.389,
        75.7757}, {-110.389, 75.7757}}, color = {0, 0, 127}));
  connect(genrou1.EFD0, genrou1.EFD) annotation (Line(points = {{-70.2,40},{-72,
          40},{-72,42.8},{-70,42.8}}, color = {0,0,127}));
  connect(gast1.dGREF, dGREF) annotation(Line(points = {{-110, 88}, {-141.645, 88}, {-141.645, 74.8072},
          {-165.039, 74.8072}, {-165.039, 74.8072}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{-60,80},{-59.9089,80},
          {-59.9089,50},{-60,50}}));
  connect(genrou1.SLIP, gast1.SLIP) annotation(Line(points = {{-50,32},{
          -36.5359,32},{-36.5359,93.9107},{-118,93.9107},{-118,72},{-110,72}}, color = {0, 0, 127}));
  connect(gast1.PMECH, genrou1.PMECH) annotation(Line(points = {{-90,88},{
          -78.7551,88},{-78.7551,46.5494},{-70,46.5494},{-70,34}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, gast1.PMECH0) annotation (Line(points = {{-70,32},{
          -81.6265,32},{-81.6265,62.4114},{-84,62.4114},{-84,86},{-90,86}},
                                        color = {0,0,127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{0,20},{-37.246,20},{
          -37.246,31.377},{-60,31.377},{-60,30}}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-60.16, 5.76}, extent = {{-116.79, 25.76}, {128.56, -28.59}}, textString = "GENROU_GAST"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN"), Text(origin = {-148.87, 69.39}, extent = {{-39.43, 24.94}, {7.19, -8.82}}, textString = "TRIP"), Text(origin = {-144.87, -82.61}, extent = {{-39.43, 24.94}, {29.19, -4.82}}, textString = "dGREF")}));
end GENROU_GAST;

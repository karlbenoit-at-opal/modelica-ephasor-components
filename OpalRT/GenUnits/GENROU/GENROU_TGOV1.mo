within OpalRT.GenUnits.GENROU;
class GENROU_TGOV1
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
  // TGOV1 Parameters
  parameter Real R_tg = 0.05 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T1_tg = 0.5 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMAX_tg = 3 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMIN_tg = 0 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T2_tg = 2.1 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T3_tg = 7 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real Dt_tg = 0 annotation(Dialog(tab = "TGOV1 Parameters"));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.TGOV1 tgov11(R = R_tg, T1 = T1_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, T2 = T2_tg, T3 = T3_tg, Dt = Dt_tg) annotation(Placement(visible = true, transformation(origin = {-20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-80, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-92.5, 72.5}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {25, 55}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-95, -65}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
equation
  connect(genrou1.VI, tgov11.VI) annotation(Line(points = {{30, 20}, {38.3925, 20},
          {38.3925, 61.2646}, {-37.0855, 61.2646}, {-37.0855, 39.8628}, {-29.8971,
          39.8628}, {-29.8971, 39.8628}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, tgov11.MBASE) annotation(Line(points = {{30, 14.6}, {40.5163,
          14.6}, {40.5163, 62.8983}, {-38.3925, 62.8983}, {-38.3925, 35.9419}, {-30.0605,
          35.9419}, {-30.0605, 35.9419}}, color = {0, 0, 127}));
  connect(tgov11.dGREF, dGREF) annotation(Line(points = {{-30, 48}, {-79.9486, 48},
          {-79.9486, 58.6118}, {-79.9486, 58.6118}}, color = {0, 0, 127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{20,10},{42.7619,10},{
          42.7619,-0.103041},{50.1809,-0.103041},{50.1809,0},{60,0}}));
  connect(genrou1.SLIP, tgov11.SLIP) annotation (Line(points = {{30,12},{
          42.4528,12},{42.4528,64.2975},{-40,64.2975},{-40,32},{-30,32}},
                                                      color = {0,0,127}));
  connect(genrou1.EFD0, genrou1.EFD) annotation (Line(points = {{9.8,20},{
          6.07941,20},{6.07941,13.9105},{10,13.9105},{10,22.8}},
                     color = {0,0,127}));
  connect(tgov11.PMECH, genrou1.PMECH) annotation(Line(points = {{-10,48},{
          -0.824327,48},{-0.824327,23.3903},{10,23.3903},{10,14}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0,tgov11.PMECH0) annotation (Line(points = {{10,12},{-6,
          12},{-6,46},{-10,46}},
                     color = {0,0,127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{25,55},{25.7602,55},{
          25.7602,40.0829},{19.8869,40.0829},{19.8869,30},{20,30}}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {1.13895, -0.455581}, extent = {{-98.1777, 97.9499}, {98.1777, -97.9499}}), Text(origin = {57.0615, -57.2869}, extent = {{25.85, -23.58}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-7.86442, 12.649}, extent = {{-73.46, 18.79}, {83.94, -29.04}}, textString = "GENROU_TGOV1"), Text(origin = {-52.2826, 72.4357}, extent = {{-23.35, 12.98}, {23.35, -12.98}}, textString = "dGREF"), Text(origin = {-53.69, -64.97}, extent = {{-23.35, 12.98}, {23.35, -12.98}}, textString = "TRIP")}));
end GENROU_TGOV1;

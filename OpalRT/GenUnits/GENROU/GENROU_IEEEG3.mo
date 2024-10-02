within OpalRT.GenUnits.GENROU;
class GENROU_IEEEG3
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENSAL Parameters"));
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
  // EXAC1 Parameters
  parameter Real TG_tg = 1 "?(>0) (sec), gate servomotor time constant";
  parameter Real TP_tg = 0.5 "(sec)";
  parameter Real Uo_tg = 0.1 "(pu/sec)";
  parameter Real Uc_tg = -0.2 "(<0)(pu/sec)";
  parameter Real PMAX_tg = 1 "(pu on machine MVA rating)";
  parameter Real PMIN_tg = 0 "(pu on machine MVA rating)";
  parameter Real Sigma_tg = 0.4 "permanent speed droop coefficient";
  parameter Real Delta_tg = 0.4 "transient speed droop coefficient";
  parameter Real TR_tg = 0.2;
  parameter Real TW_tg = 0;
  parameter Real a11_tg = 7 "(sec)";
  parameter Real a13_tg = 0.1;
  parameter Real a21_tg = 0;
  parameter Real a23_tg = 0.6 "(sec)";
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, -40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {0, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {20, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG3 ieeeg31(TG = TG_tg, TP = TP_tg, Uo = Uo_tg, Uc = Uc_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, Sigma = Sigma_tg, Delta = Delta_tg, TR = TR_tg, TW = TW_tg, a11 = a11_tg, a13 = a13_tg, a21 = a21_tg, a23 = a23_tg) annotation(Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-80, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-80, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou_base1(P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
equation
  connect(genrou_base1.VI, ieeeg31.VI) annotation(Line(points = {{35, 0}, {45.5013,
          0}, {45.5013, 51.928}, {-64.5244, 51.928}, {-64.5244, 0}, {-56.2982, 0},
          {-56.2982, 0}}, color = {0, 0, 127}));
  connect(genrou_base1.MBASE, ieeeg31.MBASE) annotation(Line(points = {{35, -8.1},
          {55.2699, -8.1}, {55.2699, 56.8123}, {-66.3239, 56.8123}, {-66.3239, -6.16967},
          {-54.4987, -6.16967}, {-54.4987, -6.16967}}, color = {0, 0, 127}));
  connect(ieeeg31.SLIP, genrou_base1.SLIP) annotation(Line(points = {{-55,-12},
          {-62.6424,-12},{-62.6424,53.9863},{48.9749,53.9863},{48.9749,
          -3.41686},{35,-3.41686},{35,-12}}, color = {0, 0, 127}));
  connect(TRIP, genrou_base1.TRIP) annotation(Line(points = {{0,40},{19.8178,40},
          {19.8178,15},{20,15}}));
  connect(genrou_base1.EFD, genrou_base1.EFD0) annotation (Line(points = {{5,4.2},
          {-0.22779,4.2},{-0.22779,-13.8952},{4.7,-13.8952},{4.7,0}},
        color = {0,0,127}));
  connect(ieeeg31.PMECH0, genrou_base1.PMECH0) annotation (Line(points = {{-25,9},
          {-39.6355,9},{-39.6355,-20.9567},{-14.123,-20.9567},{-14.123,
          -0.22779},{-3.41686,-0.22779},{-3.41686,2.05011},{5,2.05011},{5,-12}},
                  color = {0,0,127}));
  connect(genrou_base1.PMECH, ieeeg31.PMECH) annotation(Line(points = {{5,-9},{
          -10.9339,-9},{-10.9339,3.18907},{-25,3.18907},{-25,12}}, color = {0, 0, 127}));
  connect(genrou_base1.p, bus0) annotation(Line(points = {{20,-15},{46.4692,-15},
          {46.4692,-39.18},{60,-39.18},{60,-40}}));
  connect(ieeeg31.dGREF, dGREF) annotation(Line(points = {{-55, 12}, {-79.9486, 12},
         {-79.9486, -18.509}, {-79.9486, -18.509}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {1.13895, -0.455581}, extent = {{-95.8998, 87.016}, {94.0775, -92.2551}}), Text(origin = {57.0615, -57.2869}, extent = {{25.85, -23.58}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-5.13, 9.23}, extent = {{-73.46, 18.79}, {83.94, -29.04}}, textString = "GENROU_IEEEG3")}));
end GENROU_IEEEG3;

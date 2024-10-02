within OpalRT.GenUnits.GENROU;
class GENROU_IEEEG2
  parameter Real partType = 1;
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
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "IEEEG2"));
  parameter Real K_tg = 20 "K" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T1_tg = 20 "T1" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T2_tg = 20 "T2" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T3_tg = 20 "T3(>0)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real PMAX_tg = 20 "PMAX (pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real PMIN_tg = 20 "PMIN (pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T4_tg = 20 "T3(>0)" annotation(Dialog(tab = "IEEEG2"));
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG2 ieeeg21(K = K_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, PMAX = PMAX_tg, PMIN = PMIN_tg, T4 = T4_tg) annotation(Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou_base1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {0, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {80, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-80, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
    connect(genrou_base1.VI, ieeeg21.VI) annotation(Line(points = {{10, -20}, {16.3372,
          -20}, {16.3372, -33.8414}, {-58.639, -33.8414}, {-58.639, -0.437604}, {-50.0328,
          -0.437604}, {-50.0328, -0.437604}}, color = {0, 0, 127}));
  connect(genrou_base1.MBASE, ieeeg21.MBASE) annotation(Line(points = {{10, -25.4},
          {14.2951, -25.4}, {14.2951, -31.2158}, {-56.7427, -31.2158}, {-56.7427,
          -4.23017}, {-50.6162, -4.23017}, {-50.6162, -4.23017}}, color = {0, 0, 127}));
  connect(genrou_base1.TRIP, TRIP) annotation(Line(points = {{0,-10},{0,-10},{0,
          0},{0,0}}, color = {0, 0, 127}));
  connect(genrou_base1.EFD0, genrou_base1.EFD) annotation (Line(points = {{-10.2,
          -20},{-14.538,-20},{-14.538,-26.3068},{-10,-26.3068},{-10,-17.2}},
        color = {0,0,127}));
  connect(genrou_base1.p, bus0) annotation(Line(points = {{0,-30},{21.0455,-30},
          {21.0455,-39.7372},{40,-39.7372},{40,-40}}));
  connect(dGREF, ieeeg21.dGREF) annotation(Line(points = {{-80, 40}, {-79.9486, 40},
          {-79.9486, 7.71208}, {-49.8715, 7.71208}, {-49.8715, 7.71208}}));
  connect(genrou_base1.SLIP, ieeeg21.SLIP) annotation(Line(points = {{10,-28},{
          17.4435,-28},{17.4435,19.2175},{-56.6668,19.2175},{-56.6668,8.27829},
          {-50,8.27829},{-50,-8}}, color = {0, 0, 127}));
  connect(ieeeg21.PMECH, genrou_base1.PMECH) annotation(Line(points = {{-30,8},
          {-21.3856,8},{-21.3856,-16.2609},{-10,-16.2609},{-10,-26}}, color = {0, 0, 127}));
  connect(genrou_base1.PMECH0, ieeeg21.PMECH0) annotation (Line(points = {{-10,-28},
          {-53.3161,-28},{-53.3161,-3.84349},{-30,-3.84349},{-30,6}},
        color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {0, -0.35461}, extent = {{-100, 100}, {100, -100}}), Text(origin = {-30.3196, 6.20702}, extent = {{-52.66, 27.48}, {122.873, -37.0545}}, textString = "GENROU_IEEEG2")}));
end GENROU_IEEEG2;

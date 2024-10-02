within OpalRT.GenUnits.GENROU;
class GENROU_IEESGO
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 900 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 200 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.3 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.04 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.4 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 2.6 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.67 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.62 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.3 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.3 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.01 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.04 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.1 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.2 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
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
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {60, 0}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEESGO ieesgo1(T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, T4 = T4_tg, T5 = T5_tg, T6 = T6_tg, K1 = K1_tg, K2 = K2_tg, K3 = K3_tg, PMAX = PMAX_tg, PMIN = PMIN_tg) annotation(Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-80, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {20, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.VI, ieesgo1.VI) annotation(Line(points = {{85, 0}, {103.085, 0},
          {103.085, -60.4113}, {-65.0386, -60.4113}, {-65.0386, -0.257069}, {-47.3008,
          -0.257069}, {-47.3008, -0.257069}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, ieesgo1.MBASE) annotation(Line(points = {{85, -13.5}, {94.8586,
          -13.5}, {94.8586, -56.8123}, {-58.0977, -56.8123}, {-58.0977, -10.2828},
          {-46.5296, -10.2828}, {-46.5296, -10.2828}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{20,80},{60.0801,80},{
          60.0801,25},{60,25}}));
  connect(ieesgo1.dGREF, dGREF) annotation(Line(points = {{-45, 20}, {-79.9486, 20},
          {-79.9486, 77.1208}, {-79.9486, 77.1208}}, color = {0, 0, 127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{60,-25},{97.1963,-25},{
          97.1963,-80},{100,-80}}));
  connect(ieesgo1.PMECH, genrou1.PMECH) annotation(Line(points = {{5,20},{
          10.6809,20},{10.6809,7.47664},{35,7.47664},{35,-15}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, ieesgo1.PMECH0) annotation (Line(points = {{35,-20},{
          18.4246,-20},{18.4246,-38.7183},{-19.7597,-38.7183},{-19.7597,15},{
          5,15}}, color = {0,0,127}));
  connect(genrou1.EFD0, genrou1.EFD) annotation (Line(points = {{34.5,0},{
          31.7757,0},{31.7757,7},{35,7}}, color = {0,
          0,127}));
  connect(genrou1.SLIP, ieesgo1.SLIP) annotation (Line(points = {{85,-20},{92,
          -20},{92,-54},{-54,-54},{-54,-20},{-45,-20}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-68.7547, 81.7047}, extent = {{-16.42, 11.48}, {16.42, -11.48}}, textString = "TRIP"), Text(origin = {-71.1821, -78.266}, extent = {{-16.42, 11.48}, {30.8392, -12.014}}, textString = "dGREF"), Text(origin = {63.6328, -79.097}, extent = {{-16.42, 11.48}, {30.84, -12.01}}, textString = "PIN"), Text(origin = {-11.6946, 22.3452}, extent = {{-74.898, 17.0875}, {99.46, -56.34}}, textString = "GENROU_IEESGO"), Rectangle(origin = {0.267023, -0.400534}, extent = {{-99.3324, 100}, {99.3324, -100}})}));
end GENROU_IEESGO;

within OpalRT.GenUnits.GENROU;
class GENROU_SEXS
  parameter Real partType = 1;
  // constant Real pi = Modelica.Constants.pi;
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
  parameter Real S1 = 0 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // SCRX Parameters
  parameter Real TA_TB_ex = 0 "TA/TB" annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real TB_ex = 0 "(>0) (sec)" annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real K_ex = 1 annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real TE_ex = 0 "(sec)" annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real EMIN_ex = -9999 "(pu on EFD base)" annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real EMAX_ex = 9999 "(pu on EFD base)" annotation(Dialog(tab = "SEXS Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou_base1(Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12, P_gen = P_gen, Q_gen = Q_gen, IBUS = IBUS, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ID = M_ID, ZSOURCE_RE = ZSOURCE_RE) annotation(Placement(visible = true, transformation(origin={10,-26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-42,-8}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.SEXS sexs1(TA_TB = TA_TB_ex, TB = TB_ex, K = K_ex, TE = TE_ex, EMIN = EMIN_ex, EMAX = EMAX_ex, IBUS = IBUS, ID = M_ID) annotation(Placement(visible = true, transformation(origin={-16,-18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={-2,12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-72,-20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus0, genrou_base1.p)
    annotation (Line(points = {{40,-60},{10,-60},{10,-36}}, color={0,0,0}));
  connect(sexs1.EFD0, genrou_base1.EFD0)
    annotation (Line(points = {{-6,-26},{-0.2,-26}}, color={0,0,127}));
  connect(sexs1.EFD, genrou_base1.EFD) annotation (Line(points = {{-6,-24},{-4,-24},
          {-4,-23.2},{0,-23.2}}, color = {0,0,127}));
  connect(sexs1.ETERM0, genrou_base1.ETERM0) annotation (Line(points = {{-6,-22},{
          -4,-22},{-4,-21},{-0.2,-21}}, color = {0,0,127}));
  connect(sexs1.EX_AUX, genrou_base1.EX_AUX) annotation (Line(points = {{-6,-19.4},
          {-2,-19.4},{-2,-18},{-0.2,-18}}, color = {0,0,127}));
  connect(genrou_base1.VI, sexs1.VI) annotation(Line(points = {{20, -26}, {29.0488, -26},
          {29.0488, -12.3393}, {-5.39846, -12.3393}, {-5.39846, -12.3393}}, color = {0, 0, 127}));
  connect(sexs1.XADIFD, genrou_base1.XADIFD) annotation (Line(points = {{-26,-13},
          {-30,-13},{-30,-4},{26,-4},{26,-21},{20,-21}}, color = {0,0,127}));
  connect(genrou_base1.PMECH0, genrou_base1.PMECH) annotation (Line(points = {{
          0,-34},{-2,-34},{-2,-32},{0,-32}}, color = {0,0,127}));
  connect(genrou_base1.TRIP, TRIP)
    annotation (Line(points = {{10,-16},{10,12},{-2,12}}, color={0,0,127}));
  connect(sexs1.dVREF, dVREF) annotation(Line(points = {{-26, -24}, {-63.4961, -24},
          {-63.4961, -23.1362}, {-63.4961, -23.1362}}, color = {0, 0, 127}));
  connect(const.y, sexs1.VUEL) annotation (Line(points = {{-36.5,-8},{-32,-8},{
          -32,-16},{-26,-16}}, color = {0,0,127}));
  connect(sexs1.VOEL, sexs1.VUEL) annotation (Line(points = {{-26,-18.8},{-32,
          -18.8},{-32,-16},{-26,-16}}, color = {0,0,127}));
  connect(sexs1.VOTHSG, sexs1.VUEL) annotation (Line(points = {{-26,-21.6},{-32,
          -21.6},{-32,-16},{-26,-16}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-0.270636, -2.977}, extent = {{-97.6996, 76.0487}, {97.6996, -76.0487}}), Text(origin = {-64.68, -9.2}, extent = {{-20.84, 8.66}, {20.84, -8.66}}, textString = "dVREF"), Text(origin = {16.91, -8.52064}, extent = {{-30.99, 12.04}, {18.8114, -6.62728}}, textString = "TRIP")}));
end GENROU_SEXS;

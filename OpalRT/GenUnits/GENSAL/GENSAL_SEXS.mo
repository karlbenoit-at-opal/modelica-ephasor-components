within OpalRT.GenUnits.GENSAL;
class GENSAL_SEXS
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
  // SEXS Parameters
  parameter Real TA_TB_ex = 0 "TA/TB" annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real TB_ex = 0 "(>0) (sec)" annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real K_ex = 1 annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real TE_ex = 0 "(sec)" annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real EMIN_ex = -9999 "(pu on EFD base)" annotation(Dialog(tab = "SEXS Parameters"));
  parameter Real EMAX_ex = 9999 "(pu on EFD base)" annotation(Dialog(tab = "SEXS Parameters"));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {66, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-68,20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-84,6}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-60, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-18, -18}, {18, 18}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.SEXS sexs1(TA_TB = TA_TB_ex, TB = TB_ex, K = K_ex, TE = TE_ex, EMIN = EMIN_ex, EMAX = EMAX_ex, IBUS = IBUS, ID = ID) annotation(Placement(visible = true, transformation(origin={-27,15}, extent={{-19,-19},
            {19,19}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-40, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(gensal1.PMECH0, gensal1.PMECH) annotation(Line(points = {{2,-14.4},{
          0.542005,-14.4},{0.542005,-10.8},{2,-10.8}}, color = {0, 0, 127}));
  connect(gensal1.p, bus0) annotation(Line(points = {{20,-18},{20,-34.0369},{66,
          -34.0369},{66,-34}}));
  connect(TRIP, gensal1.TRIP) annotation(Line(points = {{40,80},{19.9073,80},{
          19.9073,18},{20,18}}));
  connect(gensal1.EFD0, sexs1.EFD0) annotation (Line(points = {{1.64,0},{-4,0},
          {-4,-0.2},{-8,-0.2}}, color = {0,0,127}));
  connect(sexs1.EFD, gensal1.EFD) annotation (Line(points = {{-8,3.6},{-4,3.6},
          {-4,5.04},{2,5.04}}, color = {0,0,127}));
  connect(sexs1.ETERM0, gensal1.ETERM0) annotation (Line(points = {{-8,7.4},{-3,
          7.4},{-3,9},{1.64,9}}, color = {0,0,127}));
  connect(sexs1.EX_AUX, gensal1.EX_AUX) annotation (Line(points = {{-8,12.34},{
          -4,12.34},{-4,14.4},{1.64,14.4}}, color = {0,0,127}));
  connect(sexs1.dVREF, dVREF) annotation(Line(points = {{-46, 3.6}, {-83.0334, 3.6},
         {-83.0334, 4.88432}, {-83.0334, 4.88432}}, color = {0, 0, 127}));
  connect(gensal1.VI, sexs1.VI) annotation(Line(points = {{38, 0}, {56.2982, 0},
         {56.2982, 26.7352}, {-8.74036, 26.7352}, {-8.74036, 26.7352}},
         color = {0, 0, 127}));
  connect(sexs1.XADIFD, gensal1.XADIFD) annotation (Line(points = {{-46,24.5},{
          -54,24.5},{-54,40},{48,40},{48,9},{38,9}}, color = {0,0,127}));
  connect(sexs1.VUEL, const.y) annotation (Line(points = {{-46,18.8},{-54,18.8},
          {-54,20},{-62.5,20}}, color = {0,0,127}));
  connect(sexs1.VOEL, const.y) annotation (Line(points = {{-46,13.48},{-54,
          13.48},{-54,20},{-62.5,20}}, color = {0,0,127}));
  connect(sexs1.VOTHSG, const.y) annotation (Line(points = {{-46,8.16},{-54,
          8.16},{-54,20},{-62.5,20}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-4, 2}, extent = {{-58, 29}, {58, -29}}, textString = "GENSAL_SEXS"), Text(origin = {70, -80}, extent = {{-30, 14}, {30, -14}}, textString = "PIN")}));
end GENSAL_SEXS;

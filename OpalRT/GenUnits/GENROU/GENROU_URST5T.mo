within OpalRT.GenUnits.GENROU;
class GENROU_URST5T
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
  // URST5T parameters
  parameter Real Tr_ex = 0.01 "(sec)" annotation(Dialog(tab = "URST5T Parameters"));
  parameter Real TC1_ex = 0.01 "(sec)" annotation(Dialog(tab = "URST5T Parameters"));
  parameter Real TB1_ex = 0.01 "(sec)" annotation(Dialog(tab = "URST5T Parameters"));
  parameter Real TC2_ex = 0.01 "(sec)" annotation(Dialog(tab = "URST5T Parameters"));
  parameter Real TB2_ex = 0.01 "(sec)" annotation(Dialog(tab = "URST5T Parameters"));
  parameter Real KR_ex = 0.01 annotation(Dialog(tab = "URST5T Parameters"));
  parameter Real VRMAX_ex = 0.01 annotation(Dialog(tab = "URST5T Parameters"));
  parameter Real VRMIN_ex = 0.01 annotation(Dialog(tab = "URST5T Parameters"));
  parameter Real T1_ex = 0.01 "(sec)" annotation(Dialog(tab = "URST5T Parameters"));
  parameter Real KC_ex = 0.01 annotation(Dialog(tab = "URST5T Parameters"));
  //-----------------------------------------------------
  OpalRT.Electrical.Control.Excitation.URST5T urst5t1(Tr = Tr_ex, TC1 = TC1_ex, TB1 = TB1_ex, TC2 = TC2_ex, TB2 = TB2_ex, KR = KR_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, T1 = T1_ex, KC = KC_ex) annotation(Placement(visible = true, transformation(origin={-24,40}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 20}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {97, -7}, extent = {{-7, -7}, {7, 7}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {25, 75}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-82,32}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = noVOEL) annotation(Placement(visible = true, transformation(origin={-84,50}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = noVUEL) annotation(Placement(visible = true, transformation(origin={-88,68}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {40, 100}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  parameter Real noVOEL(fixed = false, start = 1);
  parameter Real noVUEL(fixed = false, start = 1);
initial equation
  noVOEL = Modelica.Constants.inf;
  noVUEL = -Modelica.Constants.inf;
equation
  connect(genrou1.p, bus0) annotation(Line(points = {{40,-5},{93.3941,-5},{
          93.3941,-7},{97,-7}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{25,75},{40.0911,75},{
          40.0911,45},{40,45}}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{15,0},{
          11.8451,0},{11.8451,5},{15,5}}, color = {0,0,
          127}));
  connect(genrou1.VI, urst5t1.VI) annotation(Line(points = {{65, 20}, {91.5167, 20},
          {91.5167, 54.7558}, {2.05656, 54.7558}, {2.05656, 54.7558}},
          color = {0, 0, 127}));
  connect(urst5t1.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-49,52.5},
          {-58,52.5},{-58,84},{80,84},{80,32.5},{65,32.5}}, color = {0,0,127}));
  connect(urst5t1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{1,36.5},{
          8.5,36.5},{8.5,40},{14.5,40}}, color = {0,0,127}));
  connect(urst5t1.EFD0, genrou1.EFD0)
    annotation (Line(points = {{1,20},{14.5,20}}, color={0,0,127}));
  connect(urst5t1.EFD, genrou1.EFD) annotation (Line(points = {{1,25},{7.5,25},
          {7.5,27},{15,27}}, color = {0,0,127}));
  connect(urst5t1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{1,30},{8,
          30},{8,32.5},{14.5,32.5}}, color = {0,0,127}));
  connect(urst5t1.dVREF, dVREF) annotation(Line(points = {{-49, 25}, {-60.5433, 25},
          {-60.5433, 1.55239}, {-60.5433, 1.55239}}, color = {0, 0, 127}));
  connect(const.y, urst5t1.VOTHSG) annotation (Line(points = {{-75.4,32},{-62,
          32},{-62,31},{-49,31}}, color = {0,0,127}));
  connect(constant1.y, urst5t1.VOEL) annotation (Line(points = {{-77.4,50},{-72,
          50},{-72,38},{-49,38}}, color = {0,0,127}));
  connect(constant2.y, urst5t1.VUEL) annotation (Line(points = {{-81.4,68},{-64,
          68},{-64,45},{-49,45}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-1.25285, 0.113895}, extent = {{-96.9248, 95.1025}, {96.9248, -95.1025}}), Text(origin = {-9.44847, -17.8824}, extent = {{-71.18, 40.89}, {87.5809, 3.75692}}, textString = "GENROU_URST5T")}));
end GENROU_URST5T;

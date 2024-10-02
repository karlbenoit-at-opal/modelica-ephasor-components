within OpalRT.GenUnits.GENROU;
class GENROU_IEEEX1
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus";
  parameter String M_ID = "M1" "Machine Identifier";
  parameter Real P_gen = 1100 "Bus Active Power, MW";
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR";
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u.";
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg.";
  parameter Real SB = 1000 "Machine Base Power, MVA";
  parameter Real fn = 50 "Nominal frequency";
  parameter Real ZSOURCE_RE = 0 "Machine source impedence";
  parameter Real Tdo_p = 7 "d-axis transient time constant";
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s";
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s";
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s";
  parameter Real H = 50 "Inertia constant";
  parameter Real D = 0 "Speed damping";
  parameter Real Xd = 0.2 "d-axis reactance, p.u.";
  parameter Real Xq = 0.19 "q-axis reactance, p.u.";
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u.";
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u.";
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u.";
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u.";
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input";
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input";
  // // IEEEX1 Parameters
  parameter Real TR_ex = 0.025 "(sec)";
  parameter Real KA_ex = 98;
  parameter Real TA_ex = 0.2 "(sec)";
  parameter Real TB_ex = 0.5 "(sec)";
  parameter Real TC_ex = 1 "(sec)";
  parameter Real VRMAX_ex = 9 "or zero";
  parameter Real VRMIN_ex = -5;
  parameter Real KE_ex = 0.5 "or zero";
  parameter Real TE_ex = 0.35 "(>0) (sec)";
  parameter Real KF_ex = 0.01;
  parameter Real TF1_ex = 0.4 "(>0) (sec)";
  parameter Real Switch_ex = 0;
  parameter Real E1_ex = 4;
  parameter Real SE_E1_ex = 0.4;
  parameter Real E2_ex = 5;
  parameter Real SE_E2_ex = 0.5;
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEEX1 ieeex11(ID = M_ID, TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-10,30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-50,30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {5, 55}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {20, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {20, 80}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
equation
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{5,55},{19.8178,55},{
          19.8178,30},{20,30}}));
  connect(ieeex11.dVREF, dVREF) annotation(Line(points = {{-20, 24}, {-29.88, 24},
          {-29.88, 0.218103}, {-37.7317, 0.218103}, {-37.7317, 0.218103}}, color = {0, 0, 127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{20,10},{38.9231,10},{
          38.9231,-3.10144},{60,-3.10144},{60,0}}));
  connect(ieeex11.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{0,28.6},{
          6,28.6},{6,28},{9.8,28}}, color = {0,0,127}));
  connect(ieeex11.ETERM0, genrou1.ETERM0) annotation (Line(points = {{0,26},{4,
          26},{4,25},{9.8,25}}, color = {0,0,127}));
  connect(ieeex11.EFD, genrou1.EFD) annotation (Line(points = {{0,24},{6,24},{6,
          22.8},{10,22.8}}, color = {0,0,127}));
  connect(ieeex11.EFD0, genrou1.EFD0) annotation (Line(points = {{0,22},{6,22},
          {6,20},{9.8,20}}, color = {0,0,127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{10,12},{6,
          12},{6,14},{10,14}}, color = {0,0,127}));
  connect(genrou1.VI, ieeex11.VI) annotation(Line(points = {{30, 20}, {42.4165, 20},
         {42.4165, 35.9897}, {0.771208, 35.9897}, {0.771208, 35.9897}}, color = {0, 0, 127}));
  connect(ieeex11.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-20,35},{
          -26,35},{-26,46},{36,46},{36,25},{30,25}}, color = {0,0,127}));
  connect(const.y, ieeex11.VOEL) annotation (Line(points = {{-39,30},{-30,30},{
          -30,29.2},{-20,29.2}}, color = {0,0,127}));
  connect(ieeex11.VUEL, ieeex11.VOEL) annotation (Line(points = {{-20,32},{-30,
          32},{-30,29.2},{-20,29.2}}, color = {0,0,127}));
  connect(ieeex11.VOTHSG, ieeex11.VOEL) annotation (Line(points = {{-20,26.4},{
          -30,26.4},{-30,29.2},{-20,29.2}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {1.13895, -0.455581}, extent = {{-98.1777, 97.9499}, {98.1777, -97.9499}}), Text(origin = {57.0615, -57.2869}, extent = {{25.85, -23.58}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-8.32, 31.1}, extent = {{-73.46, 18.79}, {83.94, -29.04}}, textString = "GENROU_IEEEX1")}));
end GENROU_IEEEX1;

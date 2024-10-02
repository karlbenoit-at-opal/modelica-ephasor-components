within OpalRT.GenUnits.GENROU;
block GENROU_IEEET1
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
  // IEEET1 Parameters
  // This Model is located at system bus IBUS
  parameter Real TR_ex = 0.025 "(sec)";
  parameter Real KA_ex = 98;
  parameter Real TA_ex = 0.2 "(sec)";
  parameter Real VRMAX_ex = 9 "or zero";
  parameter Real VRMIN_ex = -5;
  parameter Real KE_ex = 0.5 "or zero";
  parameter Real TE_ex = 0.35 "(>0) (sec)";
  parameter Real KF_ex = 0.03;
  parameter Real TF_ex = 0.4 "(>0) (sec)";
  parameter Real Switch_ex = 0;
  parameter Real E1_ex = 4;
  parameter Real SE_E1_ex = 0.4;
  parameter Real E2_ex = 5;
  parameter Real SE_E2_ex = 0.5;
  //
  OpalRT.Electrical.Control.Excitation.IEEET1 ieeet11(ID = M_ID, TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-24,12}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-56,14}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {65, -25}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {20, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {40, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
equation
  connect(ieeet11.dVREF, dVREF) annotation(Line(points = {{-39, 3}, {-58.0437, 3}, {-58.0437, 2.00151}, {-58.0437, 2.00151}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{65,-25},{20,-25},{20,-15}}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{5,-12},{
          1.36674,-12},{1.36674,-9},{5,-9}}, color = {0,0,127}));
  connect(genrou1.TRIP, TRIP) annotation(Line(points = {{20,15},{20.2733,15},{
          20.2733,40},{20,40}}, color = {0, 0, 127}));
  connect(ieeet11.EFD0, genrou1.EFD0)
    annotation (Line(points = {{-9,0},{4.7,0}}, color={0,0,127}));
  connect(ieeet11.EFD, genrou1.EFD) annotation (Line(points = {{-9,3},{-2.5,3},
          {-2.5,4.2},{5,4.2}}, color = {0,0,127}));
  connect(ieeet11.ETERM0, genrou1.ETERM0) annotation (Line(points = {{-9,6},{-2,
          6},{-2,7.5},{4.7,7.5}}, color = {0,0,127}));
  connect(ieeet11.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-9,9.9},{
          -2.5,9.9},{-2.5,12},{4.7,12}}, color = {0,0,127}));
  connect(const.y, ieeet11.VUEL) annotation (Line(points = {{-50.5,14},{-44,14},
          {-44,15},{-39,15}}, color = {0,0,127}));
  connect(ieeet11.VOEL, ieeet11.VUEL) annotation (Line(points = {{-39,10.8},{
          -44,10.8},{-44,15},{-39,15}}, color = {0,0,127}));
  connect(ieeet11.VOTHSG, ieeet11.VUEL) annotation (Line(points = {{-39,6.6},{
          -44,6.6},{-44,15},{-39,15}}, color = {0,0,127}));
  connect(genrou1.VI, ieeet11.VI) annotation(Line(points = {{35, 0}, {48.5861, 0},
          {48.5861, 21.3368}, {-9.76864, 21.3368}, {-9.76864, 21.3368}}, color = {0, 0, 127}));
  connect(ieeet11.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-39,19.5},
          {-46,19.5},{-46,34},{42,34},{42,7.5},{35,7.5}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-1.36674, 26.4237}, extent = {{-88.3827, 36.4465}, {94.3052, -90.4328}}), Text(origin = {-31.89, -1.02}, extent = {{-48.06, 27.45}, {116.4, -21.07}}, textString = "GENROU_IEEET1")}));
end GENROU_IEEET1;

within OpalRT.GenUnits.GENROU;
class GENROU_IEEEX2
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
  // IEEEX2 Parameters
  parameter String ID = "M1" "Machine Identifier";
  parameter Real TR_ex = 0.025 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real KA_ex = 98 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real TA_ex = 0.2 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real TB_ex = 0.1 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real TC_ex = 0.2 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real VRMAX_ex = 9 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real KE_ex = 0.5 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real TE_ex = 0.35 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real KF_ex = 0.03 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real TF1_ex = 0.4 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real TF2_ex = 0.4 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "IEEEX2 Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "IEEEX2 Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {0, 20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEEX2 ieeex21(TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, TF2 = TF2_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-88,48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-150,84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-146,60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{-70,32},{
          -71.5262,32},{-71.5262,34},{-70,34}}, color = {0,0,127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{-120,80},{-59.4533,80},
          {-59.4533,50},{-60,50}}));
  connect(bus0, genrou1.p) annotation(Line(points = {{0,20},{-59.246,20},{
          -59.246,29.377},{-60,29.377},{-60,30}}));
  connect(ieeex21.EFD0, genrou1.EFD0)
    annotation (Line(points = {{-78,40},{-70.2,40}}, color={0,0,127}));
  connect(ieeex21.EFD, genrou1.EFD) annotation (Line(points = {{-78,42},{-74,42},
          {-74,42.8},{-70,42.8}}, color = {0,0,127}));
  connect(ieeex21.ETERM0, genrou1.ETERM0) annotation (Line(points = {{-78,44},{
          -74,44},{-74,45},{-70.2,45}}, color = {0,0,127}));
  connect(ieeex21.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-78,46.6},
          {-74,46.6},{-74,48},{-70.2,48}}, color = {0,0,127}));
  connect(genrou1.VI, ieeex21.VI) annotation(Line(points = {{-50, 40}, {-40.1028, 40},
          {-40.1028, 53.9846}, {-77.1208, 53.9846}, {-77.1208, 53.9846}}, color = {0, 0, 127}));
  connect(ieeex21.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-98,53},{
          -106,53},{-106,64},{-44,64},{-44,45},{-50,45}}, color = {0,0,127}));
  connect(ieeex21.dVREF, dVREF) annotation(Line(points = {{-98, 42}, {-146.272, 42}, {-146.272,
          56.5553}, {-146.272, 56.5553}}, color = {0, 0, 127}));
  connect(ieeex21.VUEL, const.y) annotation (Line(points = {{-98,50},{-112,50},
          {-112,64},{-134,64},{-134,84},{-139,84}}, color = {0,0,127}));
  connect(ieeex21.VOEL, const.y) annotation (Line(points = {{-98,47.2},{-112,
          47.2},{-112,64},{-134,64},{-134,84},{-139,84}}, color = {0,0,127}));
  connect(ieeex21.VOTHSG, const.y) annotation (Line(points = {{-98,44.4},{-112,
          44.4},{-112,64},{-134,64},{-134,84},{-139,84}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-60.1613, 5.75641}, extent = {{-72.35, 23.36}, {72.35, -23.36}}, textString = "GENROU_EXST1"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN")}), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01));
end GENROU_IEEEX2;

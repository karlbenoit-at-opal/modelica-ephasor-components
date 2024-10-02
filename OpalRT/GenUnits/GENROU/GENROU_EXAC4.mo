within OpalRT.GenUnits.GENROU;
class GENROU_EXAC4
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
  // EXAC4 Parameters
  parameter Real TR_ex = 0.01 annotation(Dialog(tab = "EXAC4 Parameters"));
  parameter Real VIMAX_ex = 0.01 annotation(Dialog(tab = "EXAC4 Parameters"));
  parameter Real VIMIN_ex = 0.01 annotation(Dialog(tab = "EXAC4 Parameters"));
  parameter Real TC_ex = 0.01 annotation(Dialog(tab = "EXAC4 Parameters"));
  parameter Real TB_ex = 0.01 "sec" annotation(Dialog(tab = "EXAC4 Parameters"));
  parameter Real KA_ex = 0.01 annotation(Dialog(tab = "EXAC4 Parameters"));
  parameter Real TA_ex = 0.01 annotation(Dialog(tab = "EXAC4 Parameters"));
  parameter Real VRMAX_ex = 0.01 annotation(Dialog(tab = "EXAC4 Parameters"));
  parameter Real VRMIN_ex = 0.01 annotation(Dialog(tab = "EXAC4 Parameters"));
  parameter Real KC_ex = 0.01 annotation(Dialog(tab = "EXAC4 Parameters"));
  OpalRT.Electrical.Control.Excitation.EXAC4 exac41(TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex) annotation(Placement(visible = true, transformation(origin={-8,12}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {30, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-58,28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {90, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-67,6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.TRIP, TRIP) annotation(Line(points = {{30,15},{29.6296,15},{
          29.6296,50},{30,50}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{90,-80},{30,-80},{30,-15}}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{15,-12},{
          13.268,-12},{13.268,-9},{15,-9}}, color = {0,0,127}));
  connect(exac41.EFD0, genrou1.EFD0)
    annotation (Line(points = {{7,0},{14.7,0}}, color={0,0,127}));
  connect(exac41.EFD, genrou1.EFD) annotation (Line(points = {{7,3},{11,3},{11,
          4.2},{15,4.2}}, color = {0,0,127}));
  connect(exac41.ETERM0, genrou1.ETERM0) annotation (Line(points = {{7,6},{12,6},
          {12,7.5},{14.7,7.5}}, color = {0,0,127}));
  connect(exac41.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{7,9.9},{
          10.5,9.9},{10.5,12},{14.7,12}}, color = {0,0,127}));
  connect(genrou1.VI, exac41.VI) annotation(Line(points = {{45, 0}, {61.9537, 0},
         {61.9537, 20.8226}, {7.96915, 20.8226}, {7.96915, 20.8226}}, color = {0, 0, 127}));
  connect(exac41.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-23,19.5},
          {-30,19.5},{-30,34},{55,34},{55,7.5},{45,7.5}}, color = {0,0,127}));
  connect(exac41.dVREF, dVREF) annotation(Line(points = {{-23, 3}, {-67.0116, 3},
          {-67.0116, 2.58732}, {-67.0116, 2.58732}}, color = {0, 0, 127}));
  connect(const.y, exac41.VUEL) annotation (Line(points = {{-47,28},{-31,28},{
          -31,15},{-23,15}}, color = {0,0,127}));
  connect(exac41.VOEL, exac41.VUEL) annotation (Line(points = {{-23,10.8},{-31,
          10.8},{-31,15},{-23,15}}, color = {0,0,127}));
  connect(exac41.VOTHSG, exac41.VUEL) annotation (Line(points = {{-23,6.6},{-31,
          6.6},{-31,15},{-23,15}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={ Text(origin = {-26.655, 37.6985}, extent = {{-64.2379, -11.0469}, {114.124, -52.2788}}, textString = "GENROU_EXAC4"), Text(origin = {-146.496, 105.781}, extent = {{57.4001, -16.517}, {114.12, -52.28}}, textString = "TRIP"), Text(origin = {-135.812, -40.7077}, extent = {{49.4273, -24.9482}, {114.12, -52.28}}, textString = "dVREF"), Text(origin = {-39.9377, -40.9545}, extent = {{80.4095, -25.4056}, {114.12, -52.28}}, textString = "PIN"), Rectangle(extent = {{-99.7722, 100}, {99.7722, -100}})}));
end GENROU_EXAC4;

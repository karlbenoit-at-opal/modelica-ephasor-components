within OpalRT.GenUnits.GENROU;
class GENROU_EXST2
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
  // EXST2 Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KA_ex = 0.2 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real TA_ex = 12 "(sec)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KE_ex = 0.2 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real TF_ex = 1.2 "(sec)" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KP_ex = 0.4 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KI_ex = 0.5 "or zero" annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "EXST2 Parameters"));
  parameter Real EFDMAX_ex = 5 annotation(Dialog(tab = "EXST2 Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXST2 exst21(TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, KP = KP_ex, KI = KI_ex, KC = KC_ex, EFDMAX = EFDMAX_ex) annotation(Placement(visible = true, transformation(origin={-14,18}, extent = {{-22.5, -22.5}, {22.5, 22.5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-62,22}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={60,-48}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={42,38}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-90,26}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.p, bus0) annotation(Line(points = {{40,-20},{39.221,-20},{
          39.221,-48},{60,-48}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{42,38},{40.1597,38},{
          40.1597,20},{40,20}}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{20,-16},{
          14,-16},{14,-12},{20,-12}}, color = {0,0,127}));
  connect(exst21.EFD0, genrou1.EFD0)
    annotation (Line(points = {{8.5,0},{19.6,0}}, color={0,0,127}));
  connect(exst21.EFD, genrou1.EFD) annotation (Line(points = {{8.5,4.5},{13.25,
          4.5},{13.25,5.6},{20,5.6}}, color = {0,0,127}));
  connect(exst21.ETERM0, genrou1.ETERM0) annotation (Line(points = {{8.5,9},{
          14.25,9},{14.25,10},{19.6,10}}, color = {0,0,127}));
  connect(exst21.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{8.5,14.85},
          {14.25,14.85},{14.25,16},{19.6,16}}, color = {0,0,127}));
    connect(genrou1.VI, exst21.VI) annotation(Line(points = {{60, 0}, {74.036, 0},
          {74.036, 47.3008}, {15.1671, 47.3008}, {15.1671, 31.8766}, {9.2545, 31.8766}, {9.2545, 31.8766}}, color = {0, 0, 127}));
  connect(exst21.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-36.5,
          29.25},{-44,29.25},{-44,54},{70,54},{70,10},{60,10}}, color = {0,0,
          127}));
  connect(exst21.dVREF, dVREF) annotation(Line(points = {{-36.5, 4.5}, {-89.9743, 4.5}, {-89.9743, 23.6504}, {-89.9743, 23.6504}}, color = {0, 0, 127}));
  connect(const.y, exst21.VUEL) annotation (Line(points = {{-56.5,22},{-46,22},
          {-46,22.5},{-36.5,22.5}}, color = {0,0,127}));
  connect(exst21.VOEL, exst21.VUEL) annotation (Line(points = {{-36.5,16.2},{
          -46,16.2},{-46,22.5},{-36.5,22.5}}, color = {0,0,127}));
  connect(exst21.VOTHSG, exst21.VUEL) annotation (Line(points = {{-36.5,9.9},{
          -46,9.9},{-46,22.5},{-36.5,22.5}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-25.4764, 17.2003}, extent = {{-63.44, 25.74}, {113.33, -34.4}}, textString = "GENROU_EXST2"), Rectangle(origin = {0.797267, -6.94761}, extent = {{-100.48, 107.066}, {99.0611, -93.1175}}), Text(origin = {-159.017, 103.015}, extent = {{71.9169, -11.991}, {113.33, -34.4}}, textString = "TRIP"), Text(origin = {-159.54, -54.6043}, extent = {{71.92, -11.99}, {122.63, -37.3892}}, textString = "dVREF"), Text(origin = {-30.9359, -53.87}, extent = {{71.92, -11.99}, {122.63, -37.39}}, textString = "PIN")}));
end GENROU_EXST2;

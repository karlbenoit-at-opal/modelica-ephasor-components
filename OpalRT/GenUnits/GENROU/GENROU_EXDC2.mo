within OpalRT.GenUnits.GENROU;
class GENROU_EXDC2
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
  // EXDC2 Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real KA_ex = 0.2 annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real TA_ex = 12 "(sec)" annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real TB_ex = 10 "(sec)" annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real TC_ex = 400 annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real KE_ex = 0.2 annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real TF1_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real Switch_ex = 0 annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "EXDC2 Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "EXDC2 Parameters"));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {38, 44}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-94, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXDC2 exdc21(TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-12,18}, extent = {{-22.5, -22.5}, {22.5, 22.5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-58,20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-60, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {60, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
equation
  connect(genrou1.p, bus0) annotation(Line(points = {{40,-20},{79.4989,-20},{
          79.4989,-20},{80,-20}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{38,44},{40.0911,44},{
          40.0911,20},{40,20}}));
  connect(exdc21.EFD0, genrou1.EFD0)
    annotation (Line(points = {{10.5,0},{19.6,0}}, color={0,0,127}));
  connect(exdc21.EFD, genrou1.EFD) annotation (Line(points = {{10.5,4.5},{15.25,
          4.5},{15.25,5.6},{20,5.6}}, color = {0,0,127}));
  connect(exdc21.ETERM0, genrou1.ETERM0) annotation (Line(points = {{10.5,9},{
          16.25,9},{16.25,10},{19.6,10}}, color = {0,0,127}));
  connect(exdc21.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{10.5,14.85},
          {16.25,14.85},{16.25,16},{19.6,16}}, color = {0,0,127}));
  connect(genrou1.VI, exdc21.VI) annotation(Line(points = {{60, 0}, {78.4062, 0},
          {78.4062, 31.8766}, {11.5681, 31.8766}, {11.5681, 31.8766}}, color = {0, 0, 127}));
  connect(exdc21.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-34.5,
          29.25},{-44,29.25},{-44,60},{70,60},{70,10},{60,10}}, color = {0,0,
          127}));
  connect(exdc21.dVREF, dVREF) annotation(Line(points = {{-34.5, 4.5}, {-60.2846, 4.5},
          {-60.2846, -21.216}, {-60.2846, -21.216}}, color = {0, 0, 127}));
  connect(const.y, exdc21.VUEL) annotation (Line(points = {{-52.5,20},{-44,20},
          {-44,22.5},{-34.5,22.5}}, color = {0,0,127}));
  connect(exdc21.VOEL, exdc21.VUEL) annotation (Line(points = {{-34.5,16.2},{
          -44,16.2},{-44,22.5},{-34.5,22.5}}, color = {0,0,127}));
  connect(exdc21.VOTHSG, exdc21.VUEL) annotation (Line(points = {{-34.5,9.9},{
          -44,9.9},{-44,22.5},{-34.5,22.5}}, color = {0,0,127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{20,-16},{
          12,-16},{12,-12},{20,-12}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-0.341686, 1.36674}, extent = {{-95.1025, 96.3554}, {95.1025, -96.3554}}), Text(origin = {-37.3558, 8.87941}, extent = {{-42.14, 24.83}, {120.044, -29.6136}}, textString = "GENROU_EXDC2")}));
end GENROU_EXDC2;

within OpalRT.GenUnits.GENROU;
class GENROU_REXSYS
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "General"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real ZSOURCE_IM = Xd_s "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
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
  //REXSYS
  parameter Real TR_ex = 0.02 "voltage transducer time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KVP_ex = 600 "voltage regulator proportional gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KVI_ex = 0.5 "voltage regulator integral gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VIMAX_ex = 0.2 "voltage regulator input limit (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TA_ex = 0.02 "voltage regulator time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TB1_ex = 1 "lag-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TC1_ex = 10 "lead-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TB2_ex = 1.0 "lag-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TC2_ex = 1.0 "lead-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VRMAX_ex = 10 "maximum controller output (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VRMIN_ex = -10 "minimum controller output (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KF_ex = 0.045 "rate feedback gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TF_ex = 5 "rate feedback >0 time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TF1_ex = 1 "feedback lead-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TF2_ex = 1 "feedback lag-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real FBF_ex = 1 "rate feedback signal flag " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KIP_ex = 5.0 "field current regulator proportional gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KII_ex = 0.5 "field current regulator integral gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TP_ex = 0.5 "field current bridge time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VFMAX_ex = 99 "maximum exciter field current (pu " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VFMIN_ex = -99 "minimum exciter field current (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KH_ex = 0.5 "field voltage controller feedback gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KE_ex = 0.4 "exciter field proportional constant" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TE_ex = 1.2 "exciter field time constant (sec >0)" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KC_ex = 0.5 "rectifier regulation factor (pu)" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KD_ex = 0.7 "exciter regulation factor (pu)" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real E1_ex = 2.4 "exciter flux at knee of curve (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real SE_E1_ex = 0.05 "saturation factor at knee" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real E2_ex = 3.2 "maximum exciter (pu)" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real SE_E2_ex = 0.3 "saturation factor at maximum flux" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real F1IMF_ex = 0.5 "power supply limit factor" annotation(Dialog(tab = "REXSYS Parameters"));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-44,18}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.REXSYS rexsys1(TR = TR_ex, KVP = KVP_ex, KVI = KVI_ex, VIMAX = VIMAX_ex, TA = TA_ex, TB1 = TB1_ex, TC1 = TC1_ex, TB2 = TB2_ex, TC2 = TC2_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KF = KF_ex, TF = TF_ex, TF1 = TF1_ex, TF2 = TF2_ex, FBF = FBF_ex, KIP = KIP_ex, KII = KII_ex, TP = TP_ex, VFMAX = VFMAX_ex, VFMIN = VFMIN_ex, KH = KH_ex, KE = KE_ex, TE = TE_ex, KC = KC_ex, KD = KD_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex, F1IMF = F1IMF_ex) annotation(Placement(visible = true, transformation(origin={-6,8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, S1 = S1, S12 = S12, Xl = Xl) annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {20, 20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-46,4}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {40, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
equation
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{20,20},{19.88,20},{
          19.88,10},{20,10}}));
  connect(genrou1.p, bus0) annotation(Line(points = {{20,-10},{20,-20},{40,-20}}));
  connect(rexsys1.EFD0, genrou1.EFD0)
    annotation (Line(points = {{4,0},{9.8,0}}, color={0,0,127}));
  connect(rexsys1.EFD, genrou1.EFD) annotation (Line(points = {{4,2},{8,2},{8,
          2.8},{10,2.8}}, color = {0,0,127}));
  connect(rexsys1.ETERM0, genrou1.ETERM0)
    annotation (Line(points = {{4,4},{8,4},{8,5},{9.8,5}}, color={0,0,127}));
  connect(rexsys1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{4,6.6},{8,
          6.6},{8,8},{9.8,8}}, color = {0,0,127}));
  connect(rexsys1.dVREF, dVREF) annotation(Line(points = {{-16, 2}, {-45.8015, 2},
          {-45.8015, 0.654308}, {-45.8015, 0.654308}}, color = {0, 0, 127}));
  connect(genrou1.VI, rexsys1.VI) annotation(Line(points = {{30, 0}, {41.3882, 0},
         {41.3882, 13.6247}, {4.37018, 13.6247}, {4.37018, 13.6247}}, color = {0, 0, 127}));
  connect(rexsys1.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-16,13},{
          -22,13},{-22,26},{36,26},{36,5},{30,5}}, color = {0,0,127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{10,-8},{8,
          -8},{8,-6},{10,-6}}, color = {0,0,127}));
  connect(const.y, rexsys1.VUEL) annotation (Line(points = {{-38.5,18},{-26,18},
          {-26,10},{-16,10}}, color = {0,0,127}));
  connect(rexsys1.VOEL, rexsys1.VUEL) annotation (Line(points = {{-16,7.2},{-20,
          7.2},{-20,10},{-16,10}}, color = {0,0,127}));
  connect(rexsys1.VOTHSG, rexsys1.VUEL) annotation (Line(points = {{-16,4.4},{
          -20,4.4},{-20,10},{-16,10}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-39.6562, -6.31756}, extent = {{-33.77, 34.64}, {121.57, -23.7468}}, textString = "GENROU_REXSYS"), Rectangle(origin = {0.108932, -0.217865}, extent = {{-99.6732, 99.7821}, {99.6732, -99.7821}}), Text(origin = {-164.763, 93.8625}, extent = {{81.2642, -1.80647}, {121.57, -23.75}}, textString = "TRIP"), Text(origin = {-162.987, -63.8121}, extent = {{81.26, -1.81}, {135.921, -28.3058}}, textString = "dVREF"), Text(origin = {-49.1386, -64.0833}, extent = {{89.6882, -1.58221}, {135.92, -28.31}}, textString = "PIN")}));
end GENROU_REXSYS;

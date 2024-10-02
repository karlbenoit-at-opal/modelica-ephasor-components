within OpalRT.GenUnits.GENSAL;
class GENSAL_EXST1_STAB4
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
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  // EXST1 Parameters
  parameter Real TR_ex = 0.02 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VIMAX_ex = 0.02 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VIMIN_ex = -0.02 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TC_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TB_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KA_ex = 500 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TA_ex = 0.01 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMAX_ex = 8 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMIN_ex = -3 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KC_ex = 0.2 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KF_ex = 0.01 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TF_ex = 0.1 annotation(Dialog(tab = "EXST1 Parameters"));
  // STAB4 Parameters
  parameter Real KX_pss = 1 "(Gain)" annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real TT_pss = 1 "Watt Transducer Time Constant" annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real TX1_pss = 1 "(> 0)" annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real TX2_pss = 1 "Reset Time Constant (> 0)" annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real Ta_pss = 1 annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real Tb_pss = 1 annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real Tc_pss = 1 "(> 0)" annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real Td_pss = 1 annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real Te_pss = 1 annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real L1_pss = -1 "Low Limit" annotation(Dialog(tab = "STAB4 Parameters"));
  parameter Real L2_pss = 1 "High Limit" annotation(Dialog(tab = "STAB4 Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {60, -20}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-40, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXST1 exst11(IBUS = IBUS, ID = ID, TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex) annotation(Placement(visible = true, transformation(origin = {17, -9}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.STAB4 stab41(KX = KX_pss, TT = TT_pss, TX1 = TX1_pss, TX2 = TX2_pss, Ta = Ta_pss, Tb = Tb_pss, Tc = Tc_pss, Td = Td_pss, Te = Te_pss, L1 = L1_pss, L2 = L2_pss) annotation(Placement(visible = true, transformation(origin = {-44, -8}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-20, -30}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {70, -48}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(gensal1.AccPower, stab41.PSS_AUX2[2]) annotation(Line(points = {{75, -23.9}, {79.6346, -23.9}, {79.6346, -59.6651}, {-62.3439, -59.6651}, {-62.3439, -17.0472}, {-59.9086, -17.0472}, {-59.9086, -17.0472}}, color = {0, 0, 127}));
  connect(gensal1.SLIP, stab41.PSS_AUX2[1]) annotation(Line(points = {{75, -32}, {79.6346, -32}, {79.6346, -59.6651}, {-62.3439, -59.6651}, {-62.3439, -17.0472}, {-60.3956, -17.0472}, {-60.3956, -17.0472}}, color = {0, 0, 127}));
  connect(stab41.VI2, gensal1.VI) annotation(Line(points = {{-59, -14}, {-65.5098, -14}, {-65.5098, -67.458}, {84.2617, -67.458}, {84.2617, -20.4566}, {75.7381, -20.4566}, {75.7381, -20.4566}}, color = {0, 0, 127}));
  connect(bus0, gensal1.p) annotation(Line(points = {{70, -48}, {60, -48}, {60, -35}}));
  connect(dVREF, exst11.dVREF) annotation(Line(points = {{-20, -30}, {-4.87062, -30}, {-4.87062, -18.0213}, {2, -18.0213}, {2, -18}}));
  connect(stab41.VI2, stab41.VI) annotation(Line(points = {{-59, -14}, {-65.7533, -14}, {-65.7533, -5.11415}, {-59.178, -5.11415}, {-59.178, -5.11415}}, color = {0, 0, 127}));
  connect(exst11.VI, gensal1.VI) annotation(Line(points = {{32, -3.33067e-16}, {79.8781, -3.33067e-16}, {79.8781, -20.2131}, {75.4946, -20.2131}, {75.4946, -20.2131}}, color = {0, 0, 127}));
  connect(gensal1.TRIP, TRIP) annotation(Line(points = {{60, -5}, {60.1705, -5}, {60.1705, 80}, {60, 80}}, color = {0, 0, 127}));
  connect(exst11.EFD0, gensal1.EFD0) annotation(Line(points = {{32, -21}, {38, -21}, {38, -20}, {44.7, -20}}, color = {0, 0, 127}));
  connect(exst11.EFD, gensal1.EFD) annotation(Line(points = {{32, -18}, {38, -18}, {38, -15.8}, {45, -15.8}}, color = {0, 0, 127}));
  connect(exst11.ETERM0, gensal1.ETERM0) annotation(Line(points = {{32, -15}, {38, -15}, {38, -12.5}, {44.7, -12.5}}, color = {0, 0, 127}));
  connect(exst11.EX_AUX, gensal1.EX_AUX) annotation(Line(points = {{32, -11.1}, {38, -11.1}, {38, -8}, {44.7, -8}}, color = {0, 0, 127}));
  connect(exst11.XADIFD, gensal1.XADIFD) annotation(Line(points = {{2, -1.5}, {-2, -1.5}, {-2, -2}, {-4, -2}, {-4, 12}, {84, 12}, {84, -12.5}, {75, -12.5}}, color = {0, 0, 127}));
  connect(exst11.VOTHSG, stab41.VOTHSG) annotation(Line(points = {{2, -14.4}, {-14, -14.4}, {-14, -17}, {-29, -17}}, color = {0, 0, 127}));
  connect(const.y, exst11.VUEL) annotation(Line(points = {{-29, 22}, {-8, 22}, {-8, -6}, {2, -6}}, color = {0, 0, 127}));
  connect(exst11.VOEL, exst11.VUEL) annotation(Line(points = {{2, -10.2}, {-8, -10.2}, {-8, -6}, {2, -6}}, color = {0, 0, 127}));
  connect(gensal1.PMECH0, gensal1.PMECH) annotation(Line(points = {{45, -32}, {40, -32}, {40, -29}, {45, -29}}, color = {0, 0, 127}));
  connect(stab41.PSS_AUX2, stab41.PSS_AUX) annotation(Line(points = {{-58.7, -17}, {-62, -17}, {-62, -8}, {-59, -8}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.341686, -0.113895}, extent = {{-99.4305, 99.4305}, {99.4305, -99.4305}}), Text(origin = {-56.15, 42.48}, extent = {{138.84, -72.32}, {-24.72, 14.46}}, textString = "GENSAL_EXST1_IEEEST"), Text(origin = {67.6574, -76.3052}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "PIN")}), experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.001));
end GENSAL_EXST1_STAB4;

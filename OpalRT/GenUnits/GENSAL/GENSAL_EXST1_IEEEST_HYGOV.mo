within OpalRT.GenUnits.GENSAL;
class GENSAL_EXST1_IEEEST_HYGOV
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
  parameter Real VIMAX_ex = 0.2 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VIMIN_ex = 0 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TC_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TB_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KA_ex = 500 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TA_ex = 0.05 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMAX_ex = 8 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real VRMIN_ex = -3 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KC_ex = 0.2 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real KF_ex = 0.1 annotation(Dialog(tab = "EXST1 Parameters"));
  parameter Real TF_ex = 1 annotation(Dialog(tab = "EXST1 Parameters"));
  // HYGOV Parameters
  parameter Real R_tg = 0.06 "Permanent Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real r_tg = 0.4 "Temporary Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tr_tg = 8 "(>0) Governor time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tf_tg = 0.05 "(>0) Filter time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tg_tg = 0.2 "(>0) Servo time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real VELM_tg = 0.01 "Gate velocity limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMAX_tg = 0.601 "Maximum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMIN_tg = 0 "Minimum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real TW_tg = 1.2 "(>0) Water time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real At_tg = 2.5 "Trubine gain" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Dturb_tg = 0 "Turbine damping" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real qNL_tg = 0.5 "No power flow" annotation(Dialog(tab = "HYGOV Parameters"));
  // IEEEST Parameters
  parameter Real A1_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A2_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A3_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A4_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A5_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A6_pss = 1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T1_pss = 1 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T2_pss = 1 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T3_pss = 1 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T4_pss = 1 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T5_pss = 7.5 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T6_pss = 7.5 "(>0)(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real KS_pss = 15.38 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMAX_pss = 0.1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMIN_pss = -0.1 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCU_pss = 2 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCL_pss = -4 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  // IEEEST ICONs
  parameter Real M0_pss = 1 "Stabilizer input code" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  parameter Real M1_pss = 0 "IB, remote bus number" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {60, -20}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {88, -50}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXST1 exst11(IBUS = IBUS, ID = ID, TR = TR_ex, VIMAX = VIMAX_ex, VIMIN = VIMIN_ex, TC = TC_ex, TB = TB_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KF = KF_ex, TF = TF_ex) annotation(Placement(visible = true, transformation(origin = {10, 6}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.HYGOV hygov1(IBUS = IBUS, ID = ID, R = R_tg, r = r_tg, Tr = Tr_tg, Tf = Tf_tg, Tg = Tg_tg, VELM = VELM_tg, GMAX = GMAX_tg, GMIN = GMIN_tg, TW = TW_tg, At = At_tg, Dturb = Dturb_tg, qNL = qNL_tg) annotation(Placement(visible = true, transformation(origin = {10, -30}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.IEEEST ieeest1(IBUS = IBUS, ID = ID, A1 = A1_pss, A2 = A2_pss, A3 = A3_pss, A4 = A4_pss, A5 = A5_pss, A6 = A6_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, KS = KS_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss) annotation(Placement(visible = true, transformation(origin = {-46, 40}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-50, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(dGREF, hygov1.dGREF) annotation(Line(points = {{-40, -40}, {-25.0837, -40}, {-25.0837, -17.5342}, {-4.62709, -17.5342}, {-4.62709, -17.5342}}));
  connect(dVREF, exst11.dVREF) annotation(Line(points = {{-40, -20}, {-27.519, -20}, {-27.519, -3.1659}, {-5.84474, -3.1659}, {-5.84474, -3.1659}}));
  connect(gensal1.AccPower, ieeest1.PSS_AUX2[2]) annotation(Line(points = {{75, -23.9}, {99.8476, -23.9}, {99.8476, -63.0745}, {-74.0334, -63.0745}, {-74.0334, 30.6849}, {-61.8568, 30.6849}, {-61.8568, 30.6849}}, color = {0, 0, 127}));
  connect(gensal1.SLIP, ieeest1.PSS_AUX2[1]) annotation(Line(points = {{75, -32}, {95.9512, -32}, {95.9512, -59.9086}, {-74.0334, -59.9086}, {-74.0334, 30.6849}, {-61.1262, 30.6849}, {-61.1262, 30.6849}}, color = {0, 0, 127}));
  connect(gensal1.VI, ieeest1.VI2) annotation(Line(points = {{75, -20}, {101.552, -20}, {101.552, -69.4063}, {-69.4063, -69.4063}, {-69.4063, 34.0943}, {-61.1262, 34.0943}, {-61.1262, 34.0943}}, color = {0, 0, 127}));
  connect(ieeest1.VI2, ieeest1.VI) annotation(Line(points = {{-61, 34}, {-69.4063, 34}, {-69.4063, 42.6179}, {-61.6133, 42.6179}, {-61.6133, 42.6179}}, color = {0, 0, 127}));
  connect(gensal1.VI, hygov1.VI) annotation(Line(points = {{75, -20}, {101.552, -20}, {101.552, -69.4063}, {-18.0213, -69.4063}, {-18.0213, -29.9543}, {-4.38356, -29.9543}, {-4.38356, -29.9543}}, color = {0, 0, 127}));
  connect(gensal1.MBASE, hygov1.MBASE) annotation(Line(points = {{75, -28.1}, {97.6559, -28.1}, {97.6559, -62.3439}, {-16.073, -62.3439}, {-16.073, -36.0426}, {-5.84474, -36.0426}, {-5.84474, -36.0426}}, color = {0, 0, 127}));
  connect(exst11.VI, gensal1.VI) annotation(Line(points = {{25, 15}, {86.2099, 15}, {86.2099, -19.9695}, {75.251, -19.9695}, {75.251, -19.9695}}, color = {0, 0, 127}));
  connect(gensal1.TRIP, TRIP) annotation(Line(points = {{60, -5}, {60.1705, -5}, {60.1705, 80}, {60, 80}}, color = {0, 0, 127}));
  connect(bus0, gensal1.p) annotation(Line(points = {{88, -50}, {60, -50}, {60, -35}}, color = {0, 0, 0}));
  connect(gensal1.PMECH0, hygov1.PMECH0) annotation(Line(points = {{45, -32}, {40, -32}, {40, -50}, {25, -50}, {25, -21}}, color = {0, 0, 127}));
  connect(hygov1.PMECH, gensal1.PMECH) annotation(Line(points = {{25, -18}, {36, -18}, {36, -29}, {45, -29}}, color = {0, 0, 127}));
  connect(gensal1.SLIP, hygov1.SLIP) annotation(Line(points = {{75, -32}, {96, -32}, {96, -60}, {-14, -60}, {-14, -42}, {-5, -42}}, color = {0, 0, 127}));
  connect(gensal1.EFD0, exst11.EFD0) annotation(Line(points = {{44.7, -20}, {34, -20}, {34, -6}, {25, -6}}, color = {0, 0, 127}));
  connect(exst11.EFD, gensal1.EFD) annotation(Line(points = {{25, -3}, {36, -3}, {36, -15.8}, {45, -15.8}}, color = {0, 0, 127}));
  connect(gensal1.ETERM0, exst11.ETERM0) annotation(Line(points = {{44.7, -12.5}, {38, -12.5}, {38, 0}, {25, 0}}, color = {0, 0, 127}));
  connect(exst11.EX_AUX, gensal1.EX_AUX) annotation(Line(points = {{25, 3.9}, {40, 3.9}, {40, -8}, {44.7, -8}}, color = {0, 0, 127}));
  connect(exst11.XADIFD, gensal1.XADIFD) annotation(Line(points = {{-5, 13.5}, {-14, 13.5}, {-14, 30}, {80, 30}, {80, -12.5}, {75, -12.5}}, color = {0, 0, 127}));
  connect(ieeest1.VOTHSG, exst11.VOTHSG) annotation(Line(points = {{-31, 31}, {-26, 31}, {-26, 0.6}, {-5, 0.6}}, color = {0, 0, 127}));
  connect(const.y, exst11.VOEL) annotation(Line(points = {{-39, 2}, {-34, 2}, {-34, 4.8}, {-5, 4.8}}, color = {0, 0, 127}));
  connect(exst11.VUEL, exst11.VOEL) annotation(Line(points = {{-5, 9}, {-34, 9}, {-34, 4.8}, {-5, 4.8}}, color = {0, 0, 127}));
  connect(ieeest1.PSS_AUX2, ieeest1.PSS_AUX) annotation(Line(points = {{-60.7, 31}, {-74, 31}, {-74, 40}, {-61, 40}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.341686, -0.113895}, extent = {{-99.4305, 99.4305}, {99.4305, -99.4305}}), Text(origin = {-56.15, 42.48}, extent = {{138.84, -72.32}, {-24.72, 14.46}}, textString = "GENSAL_EXST1_IEEEST_HYGOV"), Text(origin = {67.6574, -76.3052}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "PIN")}));
end GENSAL_EXST1_IEEEST_HYGOV;

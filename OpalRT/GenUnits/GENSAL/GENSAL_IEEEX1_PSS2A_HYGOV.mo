within OpalRT.GenUnits.GENSAL;
class GENSAL_IEEEX1_PSS2A_HYGOV
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
  // IEEEX1 Parameters
  parameter Real TR_ex = 0.025 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real KA_ex = 98 annotation(Dialog(tab = "IEEEX1"));
  parameter Real TA_ex = 0.2 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TB_ex = 0.5 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TC_ex = 1 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real VRMAX_ex = 9 "or zero" annotation(Dialog(tab = "IEEEX1"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "IEEEX1"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TE_ex = 0.35 "(>0) (sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real KF_ex = 0.03 annotation(Dialog(tab = "IEEEX1"));
  parameter Real TF1_ex = 0.4 "(>0) (sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real Switch_ex = 0 annotation(Dialog(tab = "IEEEX1"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "IEEEX1"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "IEEEX1"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "IEEEX1"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "IEEEX1"));
  // PSS2A Parameters
  parameter String PSS_ID = ID "Machine Identifier" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW1_pss = 10 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW2_pss = 10 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T6_pss = 0 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW3_pss = 10 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW4_pss = 0 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T7_pss = 10 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS2_pss = 1.13 "T7/(2*H)" annotation(Dialog(tab = "PSS2A Parameters"));
  //T7/(2*H);
  parameter Real KS3_pss = 1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T8_pss = 0.3 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T9_pss = 0.15 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS1_pss = 20 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T1_pss = 0.16 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T2_pss = 0.02 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T3_pss = 0.16 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T4_pss = 0.02 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMAX_pss = 0.2 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMIN_pss = -0.066 annotation(Dialog(tab = "PSS2A Parameters"));
  // PSS2A ICONs
  parameter Real M0_pss = 1 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M1_pss = 0 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M2_pss = 0 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M3_pss = 0 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M4_pss = 4 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M5_pss = 2 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  //
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
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {60, -20}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {100, -80}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEEX1 ieeex11(TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {11, -3}, extent = {{-21, -21}, {21, 21}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-40, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2A pSS2A1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = PSS_ID) annotation(Placement(visible = true, transformation(origin = {-44, -10}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.HYGOV hygov1(R = R_tg, r = r_tg, Tr = Tr_tg, Tf = Tf_tg, Tg = Tg_tg, VELM = VELM_tg, GMAX = GMAX_tg, GMIN = GMIN_tg, TW = TW_tg, At = At_tg, Dturb = Dturb_tg, qNL = qNL_tg, IBUS = IBUS, ID = ID) annotation(Placement(visible = true, transformation(origin = {16, -42}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-47, -29}, extent = {{-7, -7}, {7, 7}}, rotation = 0), iconTransformation(origin = {-40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-47, -47}, extent = {{-7, -7}, {7, 7}}, rotation = 0), iconTransformation(origin = {-40, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(dGREF, hygov1.dGREF) annotation(Line(points = {{-47, -47}, {-21.4307, -47}, {-21.4307, -28.4931}, {0, -28.4931}, {0, -28.4931}}));
  connect(dVREF, ieeex11.dVREF) annotation(Line(points = {{-47, -29}, {-16.073, -29}, {-16.073, -16.073}, {-11.4459, -16.073}, {-11.4459, -16.073}}));
  connect(gensal1.VI, pSS2A1.VI2) annotation(Line(points = {{75, -20}, {86.697, -20}, {86.697, -76.7122}, {-64.0486, -76.7122}, {-64.0486, -13.8813}, {-59.178, -13.8813}, {-59.178, -13.8813}}, color = {0, 0, 127}));
  connect(gensal1.AccPower, pSS2A1.PSS_AUX2[2]) annotation(Line(points = {{75, -23.9}, {82.0699, -23.9}, {82.0699, -72.0851}, {-65.9969, -72.0851}, {-65.9969, -16.073}, {-59.178, -16.073}, {-59.178, -16.073}}, color = {0, 0, 127}));
  connect(gensal1.SLIP, pSS2A1.PSS_AUX2[1]) annotation(Line(points = {{75, -32}, {82.0699, -32}, {82.0699, -72.0851}, {-65.9969, -72.0851}, {-65.9969, -16.073}, {-60.1521, -16.073}, {-60.1521, -16.073}}, color = {0, 0, 127}));
  connect(pSS2A1.VI2, pSS2A1.VI) annotation(Line(points = {{-59, -14}, {-64.0486, -14}, {-64.0486, -8.03652}, {-59.4215, -8.03652}, {-59.4215, -8.03652}}, color = {0, 0, 127}));
  connect(ieeex11.VI, gensal1.VI) annotation(Line(points = {{32, 9.6}, {86.697, 9.6}, {86.697, -19.9695}, {75.7381, -19.9695}, {75.7381, -19.9695}}, color = {0, 0, 127}));
  connect(gensal1.MBASE, hygov1.MBASE) annotation(Line(points = {{75, -28.1}, {84.7487, -28.1}, {84.7487, -74.2769}, {-10.4718, -74.2769}, {-10.4718, -48.4626}, {-1.46119, -48.4626}, {-1.46119, -48.4626}}, color = {0, 0, 127}));
  connect(gensal1.VI, hygov1.VI) annotation(Line(points = {{75, -20}, {86.697, -20}, {86.697, -76.7122}, {-14.3683, -76.7122}, {-14.3683, -41.6438}, {-1.70472, -41.6438}, {-1.70472, -41.6438}}, color = {0, 0, 127}));
  connect(hygov1.PMECH0, gensal1.PMECH0) annotation(Line(points = {{33, -31.8}, {44.0791, -31.8}, {44.0791, -31.659}, {44.0791, -31.659}}, color = {0, 0, 127}));
  connect(hygov1.PMECH, gensal1.PMECH) annotation(Line(points = {{33, -28.4}, {43.592, -28.4}, {43.592, -28.006}, {43.592, -28.006}}, color = {0, 0, 127}));
  connect(hygov1.SLIP, gensal1.SLIP) annotation(Line(points = {{-1, -55.6}, {-8, -55.6}, {-8, -72}, {82, -72}, {82, -32}, {75, -32}}, color = {0, 0, 127}));
  connect(bus0, gensal1.p) annotation(Line(points = {{100, -80}, {59.9813, -80}, {59.9813, -33.9119}, {60, -33.9119}, {60, -35}}));
  connect(gensal1.TRIP, TRIP) annotation(Line(points = {{60, -5}, {60.1705, -5}, {60.1705, 80}, {60, 80}}, color = {0, 0, 127}));
  connect(ieeex11.EFD0, gensal1.EFD0) annotation(Line(points = {{32, -19.8}, {39, -19.8}, {39, -20}, {44.7, -20}}, color = {0, 0, 127}));
  connect(ieeex11.EFD, gensal1.EFD) annotation(Line(points = {{32, -15.6}, {38, -15.6}, {38, -15.8}, {45, -15.8}}, color = {0, 0, 127}));
  connect(ieeex11.ETERM0, gensal1.ETERM0) annotation(Line(points = {{32, -11.4}, {39, -11.4}, {39, -12.5}, {44.7, -12.5}}, color = {0, 0, 127}));
  connect(ieeex11.EX_AUX, gensal1.EX_AUX) annotation(Line(points = {{32, -5.94}, {39, -5.94}, {39, -8}, {44.7, -8}}, color = {0, 0, 127}));
  connect(ieeex11.XADIFD, gensal1.XADIFD) annotation(Line(points = {{-10, 7.5}, {-18, 7.5}, {-18, 24}, {82, 24}, {82, -12.5}, {75, -12.5}}, color = {0, 0, 127}));
  connect(pSS2A1.VOTHSG, ieeex11.VOTHSG) annotation(Line(points = {{-29, -16}, {-20, -16}, {-20, -10.56}, {-10, -10.56}}, color = {0, 0, 127}));
  connect(const.y, ieeex11.VUEL) annotation(Line(points = {{-29, 16}, {-22, 16}, {-22, 1.2}, {-10, 1.2}}, color = {0, 0, 127}));
  connect(ieeex11.VOEL, ieeex11.VUEL) annotation(Line(points = {{-10, -4.68}, {-22, -4.68}, {-22, 1.2}, {-10, 1.2}}, color = {0, 0, 127}));
  connect(pSS2A1.PSS_AUX2, pSS2A1.PSS_AUX) annotation(Line(points = {{-58.7, -16}, {-66, -16}, {-66, -10}, {-59, -10}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"), experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.001), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.341686, -0.113895}, extent = {{-99.4305, 99.4305}, {99.4305, -99.4305}}), Text(origin = {-54.15, 26.48}, extent = {{138.84, -72.32}, {-24.72, 14.46}}, textString = "GENSAL_IEEEX1_PSS2A_HYGOV"), Text(origin = {67.6574, -76.3052}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "PIN")}));
end GENSAL_IEEEX1_PSS2A_HYGOV;

within OpalRT.GenUnits.GENSAL;
class GENSAL_IEEEX1_STAB1_HYGOV
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
  parameter Real TB_ex = 0.01 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TC_ex = 0.01 "(sec)" annotation(Dialog(tab = "IEEEX1"));
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
  // HYGOV Parameters
  parameter Real R_tg = 0.06 "Permanent Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real r_tg = 0.4 "Temporary Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tr_tg = 8 "(>0) Governor time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tf_tg = 0.5 "(>0) Filter time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tg_tg = 0.02 "(>0) Servo time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real VELM_tg = 0.01 "Gate velocity limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMAX_tg = 0.06 "Maximum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMIN_tg = 0.2 "Minimum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real TW_tg = 1.2 "(>0) Water time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real At_tg = 2.5 "Trubine gain" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Dturb_tg = 0 "Turbine damping" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real qNL_tg = 0.5 "No power flow" annotation(Dialog(tab = "HYGOV Parameters"));
  // STAB Parameters
  parameter Real K_T_pss = 0.01 "sec^-1" annotation(Dialog(tab = "STAB1 Parameters"));
  parameter Real T_pss = 0.01 "sec" annotation(Dialog(tab = "STAB1 Parameters"));
  parameter Real T1_T3_pss = 0.01 annotation(Dialog(tab = "STAB1 Parameters"));
  parameter Real T3_pss = 0.01 "sec" annotation(Dialog(tab = "STAB1 Parameters"));
  parameter Real T2_T4_pss = 0.01 annotation(Dialog(tab = "STAB1 Parameters"));
  parameter Real T4_pss = 0.01 "sec" annotation(Dialog(tab = "STAB1 Parameters"));
  parameter Real HLIM_pss = 0.01 "high limit" annotation(Dialog(tab = "STAB1 Parameters"));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.HYGOV hygov1(R = R_tg, r = r_tg, Tr = Tr_tg, Tf = Tf_tg, Tg = Tg_tg, VELM = VELM_tg, GMAX = GMAX_tg, GMIN = GMIN_tg, TW = TW_tg, At = At_tg, Dturb = Dturb_tg, qNL = qNL_tg, IBUS = IBUS, ID = ID) annotation(Placement(visible = true, transformation(origin = {12, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {86, -26}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-18, 34}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEEX1 ieeex11(TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {12, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {45, 35}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {40, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.STAB1 stab11(K_T = K_T_pss, T = T_pss, T1_T3 = T1_T3_pss, T3 = T3_pss, T2_T4 = T2_T4_pss, T4 = T4_pss, HLIM = HLIM_pss) annotation(Placement(visible = true, transformation(origin = {-18, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-20, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {60, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
equation
  connect(hygov1.dGREF, dGREF) annotation(Line(points = {{2, -8}, {-19.9314, -8}, {-19.9314, -19.9314}, {-19.9314, -19.9314}}, color = {0, 0, 127}));
  connect(ieeex11.dVREF, dVREF) annotation(Line(points = {{2, 2}, {-1.79709, 2}, {-1.79709, -2.9407}, {-37.4122, -2.9407}, {-37.4122, -2.9407}}, color = {0, 0, 127}));
  connect(gensal1.VI, hygov1.VI) annotation(Line(points = {{50, 0}, {60.627, 0}, {60.627, -36.7099}, {-5.1913, -36.7099}, {-5.1913, -16.0374}, {1.76133, -16.0374}, {1.76133, -16.0374}}, color = {0, 0, 127}));
  connect(gensal1.MBASE, hygov1.MBASE) annotation(Line(points = {{50, -5.4}, {58.6802, -5.4}, {58.6802, -34.6705}, {-3.52267, -34.6705}, {-3.52267, -20.0236}, {2.03944, -20.0236}, {2.03944, -20.0236}}, color = {0, 0, 127}));
  connect(stab11.VI, stab11.VI2) annotation(Line(points = {{-28, 10}, {-32.9662, 10}, {-32.9662, 3.93844}, {-28.736, 3.93844}, {-28.736, 3.93844}}, color = {0, 0, 127}));
  connect(gensal1.VI, stab11.VI) annotation(Line(points = {{50, 0}, {55.1381, 0}, {55.1381, 24.9434}, {-32.9662, 24.9434}, {-32.9662, 10.0649}, {-28.5901, 10.0649}, {-28.5901, 10.0649}}, color = {0, 0, 127}));
  connect(TRIP, gensal1.TRIP) annotation(Line(points = {{45, 35}, {40.0911, 35}, {40.0911, 10}, {40, 10}}));
  //
  connect(gensal1.p, bus0) annotation(Line(points = {{40, -10}, {40, -26}, {86, -26}}));
  connect(ieeex11.EFD0, gensal1.EFD0) annotation(Line(points = {{22, 0}, {29.8, 0}}, color = {0, 0, 127}));
  connect(ieeex11.EFD, gensal1.EFD) annotation(Line(points = {{22, 2}, {26, 2}, {26, 2.8}, {30, 2.8}}, color = {0, 0, 127}));
  connect(ieeex11.ETERM0, gensal1.ETERM0) annotation(Line(points = {{22, 4}, {26, 4}, {26, 5}, {29.8, 5}}, color = {0, 0, 127}));
  connect(ieeex11.EX_AUX, gensal1.EX_AUX) annotation(Line(points = {{22, 6.6}, {26, 6.6}, {26, 8}, {29.8, 8}}, color = {0, 0, 127}));
  connect(stab11.VOTHSG, ieeex11.VOTHSG) annotation(Line(points = {{-8, 2}, {-4, 2}, {-4, 4.4}, {2, 4.4}}, color = {0, 0, 127}));
  connect(hygov1.PMECH, gensal1.PMECH) annotation(Line(points = {{22, -8}, {24, -8}, {24, -6}, {30, -6}}, color = {0, 0, 127}));
  connect(gensal1.PMECH0, hygov1.PMECH0) annotation(Line(points = {{30, -8}, {26, -8}, {26, -28}, {22, -28}, {22, -10}}, color = {0, 0, 127}));
  connect(hygov1.SLIP, gensal1.SLIP) annotation(Line(points = {{2, -24}, {-2, -24}, {-2, -32}, {56, -32}, {56, -8}, {50, -8}}, color = {0, 0, 127}));
  connect(gensal1.VI, ieeex11.VI) annotation(Line(points = {{50, 0}, {55.1381, 0}, {55.1381, 14.0033}, {22.6096, 14.0033}, {22.6096, 14.0033}}, color = {0, 0, 127}));
  connect(ieeex11.XADIFD, gensal1.XADIFD) annotation(Line(points = {{2, 13}, {-2, 13}, {-2, 24}, {54, 24}, {54, 5}, {50, 5}}, color = {0, 0, 127}));
  connect(stab11.PSS_AUX[1], gensal1.SLIP) annotation(Line(points = {{-28, 8}, {-34.0216, 8}, {-34.0216, 26.0492}, {56.4554, 26.0492}, {56.4554, -7.97236}, {49.5955, -7.97236}, {49.5955, -7.97236}}, color = {0, 0, 127}));
  connect(stab11.PSS_AUX[2], gensal1.AccPower) annotation(Line(points = {{-28, 8}, {-34.0216, 8}, {-34.0216, 26.0492}, {56.4554, 26.0492}, {56.4554, -2.59565}, {49.6882, -2.59565}, {49.6882, -2.59565}}, color = {0, 0, 127}));
  connect(const.y, ieeex11.VUEL) annotation(Line(points = {{-12.5, 34}, {-4, 34}, {-4, 10}, {2, 10}}, color = {0, 0, 127}));
  connect(ieeex11.VOEL, ieeex11.VUEL) annotation(Line(points = {{2, 7.2}, {0, 7.2}, {0, 10}, {2, 10}}, color = {0, 0, 127}));
  connect(stab11.PSS_AUX2, stab11.PSS_AUX) annotation(Line(points = {{-27.8, 2}, {-34, 2}, {-34, 8}, {-28, 8}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.341686, -0.113895}, extent = {{-99.4305, 99.4305}, {99.4305, -99.4305}}), Text(origin = {-57.06, 28.58}, extent = {{143.62, -49.31}, {-24.72, 14.46}}, textString = "GENSAL_IEEEX1_STAB1_HYGOV"), Text(origin = {67.6574, -76.3052}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "PIN")}));
end GENSAL_IEEEX1_STAB1_HYGOV;

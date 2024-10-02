within OpalRT.GenUnits.GENROU;
model GENROU_EXPIC1_GAST
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
  // EXPIC1 Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KA_ex = 0.2 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TA1_ex = 12 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real VR1_ex = 5 "or zero" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real VR2_ex = -5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TA2_ex = 10 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TA3_ex = 400 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TA4_ex = 5 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TF1_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TF2_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real EFDMAX_ex = 5 "or zero" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real EFDMIN_ex = -5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KE_ex = 0.2 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KP_ex = 0.4 annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KI_ex = 0.5 "or zero" annotation(Dialog(tab = "EXPIC1 Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "EXPIC1 Parameters"));
  // GAST Parameters
  parameter Real R_tg = 0.047 "Speed droop" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T1_tg = 0.4 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T2_tg = 0.1 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T3_tg = 3 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real AT_tg = 1.5 "Ambient temperature load limit" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real KT_tg = 2 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMAX_tg = 1.5 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMIN_tg = 0 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real DTURB_tg = 0 annotation(Dialog(tab = "GAST Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.EXPIC1 expic11(TR = TR_ex, KA = KA_ex, TA1 = TA1_ex, VR1 = VR1_ex, VR2 = VR2_ex, TA2 = TA2_ex, TA3 = TA3_ex, TA4 = TA4_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KF = KF_ex, TF1 = TF1_ex, TF2 = TF2_ex, EFDMAX = EFDMAX_ex, EFDMIN = EFDMIN_ex, KE = KE_ex, TE = TE_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex, KP = KP_ex, KI = KI_ex, KC = KC_ex) annotation(Placement(visible = true, transformation(origin = {-14, 18}, extent = {{-22.5, -22.5}, {22.5, 22.5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-62, 30}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.GAST gast1(R = R_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, AT = AT_tg, KT = KT_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, DTURB = DTURB_tg) annotation(Placement(visible = true, transformation(origin = {-10, -28}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-82, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {38, 44}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-82, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(expic11.VI, genrou1.VI) annotation(Line(points = {{8.5, 31.5}, {82.8326, 31.5}, {82.8326, -0.429185}, {61.8026, -0.429185}, {61.8026, -0.429185}}, color = {0, 0, 127}));
  connect(gast1.MBASE, genrou1.MBASE) annotation(Line(points = {{-30, -36}, {-42.4893, -36}, {-42.4893, -59.6567}, {68.6695, -59.6567}, {68.6695, -11.588}, {60.515, -11.588}, {60.515, -11.588}}, color = {0, 0, 127}));
  connect(gast1.VI, genrou1.VI) annotation(Line(points = {{-30, -28}, {-45.9227, -28}, {-45.9227, -64.3777}, {76.824, -64.3777}, {76.824, 0}, {61.8026, 0}, {61.8026, 0}}, color = {0, 0, 127}));
  connect(gast1.PMECH0, genrou1.PMECH0) annotation(Line(points = {{10, -16}, {19.7425, -16}, {19.7425, -16.309}, {19.7425, -16.309}}, color = {0, 0, 127}));
  connect(dGREF, gast1.dGREF) annotation(Line(points = {{-82, -14}, {-46.3519, -14}, {-46.3519, -12.4464}, {-30.9013, -12.4464}, {-30.9013, -12.4464}}));
  connect(dVREF, expic11.dVREF) annotation(Line(points = {{-82, 8}, {-61.3734, 8}, {-61.3734, 4.29185}, {-36.4807, 4.29185}, {-36.4807, 4.29185}}));
  connect(genrou1.p, bus0) annotation(Line(points = {{40, -20}, {77.221, -20}, {77.221, -20}, {80, -20}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{38, 44}, {38.1597, 44}, {38.1597, 20}, {40, 20}}));
  connect(expic11.EFD0, genrou1.EFD0) annotation(Line(points = {{8.5, 0}, {19.6, 0}}, color = {0, 0, 127}));
  connect(expic11.EFD, genrou1.EFD) annotation(Line(points = {{8.5, 4.5}, {14.25, 4.5}, {14.25, 5.6}, {20, 5.6}}, color = {0, 0, 127}));
  connect(expic11.ETERM0, genrou1.ETERM0) annotation(Line(points = {{8.5, 9}, {14.25, 9}, {14.25, 10}, {19.6, 10}}, color = {0, 0, 127}));
  connect(expic11.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{8.5, 14.85}, {14.25, 14.85}, {14.25, 16}, {19.6, 16}}, color = {0, 0, 127}));
  connect(expic11.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-36.5, 29.25}, {-46, 29.25}, {-46, 54}, {74, 54}, {74, 10}, {60, 10}}, color = {0, 0, 127}));
  connect(gast1.PMECH, genrou1.PMECH) annotation(Line(points = {{10, -12}, {20, -12}}, color = {0, 0, 127}));
  connect(const.y, expic11.VUEL) annotation(Line(points = {{-56.5, 30}, {-50, 30}, {-50, 22.5}, {-36.5, 22.5}}, color = {0, 0, 127}));
  connect(expic11.VOEL, expic11.VUEL) annotation(Line(points = {{-36.5, 16.2}, {-50, 16.2}, {-50, 22.5}, {-36.5, 22.5}}, color = {0, 0, 127}));
  connect(expic11.VOTHSG, expic11.VUEL) annotation(Line(points = {{-36.5, 9.9}, {-50, 9.9}, {-50, 22.5}, {-36.5, 22.5}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, gast1.SLIP) annotation(Line(points = {{60, -16}, {64, -16}, {64, -56}, {-38, -56}, {-38, -44}, {-30, -44}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-25.4764, 17.2003}, extent = {{-63.44, 25.74}, {113.33, -34.4}}, textString = "GENROU_EXPIC1_GAST"), Rectangle(origin = {0.797267, -6.94761}, extent = {{-100.48, 107.066}, {99.0611, -93.1175}}), Text(origin = {-159.017, 103.015}, extent = {{71.9169, -11.991}, {113.33, -34.4}}, textString = "TRIP"), Text(origin = {-159.274, -6.51707}, extent = {{71.92, -11.99}, {118.312, -35.7285}}, textString = "dGREF"), Text(origin = {-159.54, -54.6043}, extent = {{71.92, -11.99}, {122.63, -37.3892}}, textString = "dVREF"), Text(origin = {-30.9359, -53.87}, extent = {{71.92, -11.99}, {122.63, -37.39}}, textString = "PIN")}));
end GENROU_EXPIC1_GAST;

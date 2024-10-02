within OpalRT.GenUnits.GENSAL;
model GENSAL_IEEET1_MAXEX2
  parameter Real partType = 1;
  // GENSAL Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tdo_p = 10 "d-axis transient time constant" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tdo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tqo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S1 = 0 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S12 = 0 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  // IEEET1 Parameters
  // This Model is located at system bus IBUS
  parameter Real TR_ex = 0.025 "(sec)";
  parameter Real KA_ex = 98;
  parameter Real TA_ex = 0.2 "(sec)";
  parameter Real VRMAX_ex = 9 "or zero";
  parameter Real VRMIN_ex = -5;
  parameter Real KE_ex = 0.5 "or zero";
  parameter Real TE_ex = 0.35 "(>0) (sec)";
  parameter Real KF_ex = 0.03;
  parameter Real TF_ex = 0.4 "(>0) (sec)";
  parameter Real Switch_ex = 0;
  parameter Real E1_ex = 4;
  parameter Real SE_E1_ex = 0.4;
  parameter Real E2_ex = 5;
  parameter Real SE_E2_ex = 0.5;
  //
  // MAXEX2 Parameters
  parameter Real EFDRATED_OEL = 2.8 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real EFD1_OEL = 1.2 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real TIME1_OEL = 50 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real EFD2_OEL = 1.3 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real TIME2_OEL = 30 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real EFD3_OEL = 1.5 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real TIME3_OEL = 5 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real EFDDES_OEL = 1 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real KMX_OEL = 1.5 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real VLOW_OEL = -0.1 annotation(Dialog(tab = "MAXEX2 Parameters"));
  parameter Real ICONM_OEL = 0 annotation(Dialog(tab = "MAXEX2 Parameters", group = "ICONs"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 6.21725e-15}, extent = {{-17.5, -17.5}, {17.5, 17.5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEET1 ieeet11(ID = M_ID, TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {-30.5, 15.5}, extent = {{-20.5, -20.5}, {20.5, 20.5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {65, -25}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {20, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.OverExcitationLimiter.MAXEX2 maxex21(EFDRATED = EFDRATED_OEL, EFD1 = EFD1_OEL, TIME1 = TIME1_OEL, EFD2 = EFD2_OEL, TIME2 = TIME2_OEL, EFD3 = EFD3_OEL, TIME3 = TIME3_OEL, EFDDES = EFDDES_OEL, KMX = KMX_OEL, VLOW = VLOW_OEL, ICONM = ICONM_OEL) annotation(Placement(visible = true, transformation(origin = {-76, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-60, 20}, extent = {{-2.5, -2.5}, {2.5, 2.5}}, rotation = 0)));
equation
  connect(ieeet11.VI, gensal1.VI) annotation(Line(points = {{-10, 27.8}, {54.577, 27.8}, {54.577, 0.217438}, {39.1389, 0.217438}, {39.1389, 0.217438}}, color = {0, 0, 127}));
  connect(const.y, ieeet11.VUEL) annotation(Line(points = {{-57.25, 20}, {-54.875, 20}, {-54.875, 19.6}, {-51, 19.6}}, color = {0, 0, 127}));
  connect(dVREF, ieeet11.dVREF) annotation(Line(points = {{-80, 0}, {-66.1012, 0}, {-66.1012, 2.60926}, {-51.5329, 2.60926}, {-51.5329, 2.60926}}));
  connect(gensal1.PMECH0, gensal1.PMECH) annotation(Line(points = {{2.5, -14}, {1.35501, -14}, {1.35501, -10.5}, {2.5, -10.5}}, color = {0, 0, 127}));
  connect(gensal1.p, bus0) annotation(Line(points = {{20, -17.5}, {20, -24}, {65, -24}, {65, -25}}));
  connect(TRIP, gensal1.TRIP) annotation(Line(points = {{20, 40}, {20.0456, 40}, {20.0456, 17.5}, {20, 17.5}}));
  connect(ieeet11.EFD0, gensal1.EFD0) annotation(Line(points = {{-10, -0.9}, {-4, -0.9}, {-4, 0}, {2.15, 0}}, color = {0, 0, 127}));
  connect(ieeet11.EFD, gensal1.EFD) annotation(Line(points = {{-10, 3.2}, {-4, 3.2}, {-4, 4.9}, {2.5, 4.9}}, color = {0, 0, 127}));
  connect(ieeet11.ETERM0, gensal1.ETERM0) annotation(Line(points = {{-10, 7.3}, {-4, 7.3}, {-4, 8.75}, {2.15, 8.75}}, color = {0, 0, 127}));
  connect(ieeet11.EX_AUX, gensal1.EX_AUX) annotation(Line(points = {{-10, 12.63}, {-4, 12.63}, {-4, 14}, {2.15, 14}}, color = {0, 0, 127}));
  connect(ieeet11.XADIFD, gensal1.XADIFD) annotation(Line(points = {{-51, 25.75}, {-62, 25.75}, {-62, 50}, {50, 50}, {50, 8.75}, {37.5, 8.75}}, color = {0, 0, 127}));
  connect(maxex21.VOEL, ieeet11.VOEL) annotation(Line(points = {{-65.6, 20}, {-58, 20}, {-58, 13.86}, {-51, 13.86}}, color = {0, 0, 127}));
  connect(ieeet11.VOTHSG, ieeet11.VUEL) annotation(Line(points = {{-51, 8.12}, {-54, 8.12}, {-54, 8}, {-56, 8}, {-56, 19.6}, {-51, 19.6}}, color = {0, 0, 127}));
  connect(maxex21.XADIFD, gensal1.XADIFD) annotation(Line(points = {{-85.8, 14}, {-88, 14}, {-88, 50}, {50, 50}, {50, 8.75}, {37.5, 8.75}}, color = {0, 0, 127}));
  connect(maxex21.EFD, gensal1.EFD) annotation(Line(points = {{-85.8, 26}, {-90, 26}, {-90, -10}, {-4, -10}, {-4, 4.9}, {2.5, 4.9}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {-1.36674, 26.4237}, extent = {{-88.3827, 36.4465}, {94.3052, -90.4328}}), Text(origin = {-31.8919, -1.02164}, extent = {{-48.06, 27.45}, {116.397, -21.0719}}, textString = "GENSAL_IEEET1")}));
end GENSAL_IEEET1_MAXEX2;

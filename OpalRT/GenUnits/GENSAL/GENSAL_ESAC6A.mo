within OpalRT.GenUnits.GENSAL;
class GENSAL_ESAC6A
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
  // ESAC6A parameters
  parameter Real TR_ex = 0.02;
  parameter Real KA_ex = 536;
  parameter Real TA_ex = 0.086;
  parameter Real TK_ex = 0.18;
  parameter Real TB_ex = 9;
  parameter Real TC_ex = 3;
  parameter Real VAMAX_ex = 75;
  parameter Real VAMIN_ex = -75;
  parameter Real VRMAX_ex = 44;
  parameter Real VRMIN_ex = -36;
  parameter Real TE_ex = 1;
  parameter Real VFELIM_ex = 19;
  parameter Real KH_ex = 92;
  parameter Real VHMAX_ex = 75;
  parameter Real TH_ex = 0.08;
  parameter Real TJ_ex = 0.02;
  parameter Real KC_ex = 0.173;
  parameter Real KD_ex = 1.91;
  parameter Real KE_ex = 1.6;
  parameter Real E1_ex = 7.4;
  parameter Real SE_E1_ex = 0.214;
  parameter Real E2_ex = 5.55;
  parameter Real SE_E2_ex = 0.044;
  //
  //****************************
  //
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {75, -25}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //OpalRT.Connector.InterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 35}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-20, 16}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESAC6A esac6a1(TR = TR_ex, KA = KA_ex, TA = TA_ex, TK = TK_ex, TB = TB_ex, TC = TC_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, TE = TE_ex, VFELIM = VFELIM_ex, KH = KH_ex, VHMAX = VHMAX_ex, TH = TH_ex, TJ = TJ_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {12, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-30, 4}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(esac6a1.VI, gensal1.VI) annotation(Line(points = {{22, 14}, {53.7046, 14}, {53.7046, -0.464305}, {50.4544, -0.464305}, {50.4544, -0.464305}}, color = {0, 0, 127}));
  connect(dVREF, esac6a1.dVREF) annotation(Line(points = {{-30, 4}, {-18.4174, 4}, {-18.4174, 1.54768}, {1.39291, 1.54768}, {1.39291, 1.54768}}));
  connect(gensal1.PMECH0, gensal1.PMECH) annotation(Line(points = {{30, -8}, {28.97, -8}, {28.97, -6}, {30, -6}}, color = {0, 0, 127}));
  connect(TRIP, gensal1.TRIP) annotation(Line(points = {{40, 35}, {40.1656, 35}, {40.1656, 10}, {40, 10}}));
  connect(gensal1.p, bus0) annotation(Line(points = {{40, -10}, {40, -24}, {75, -24}, {75, -25}}));
  connect(esac6a1.EFD0, gensal1.EFD0) annotation(Line(points = {{22, 0}, {29.8, 0}}, color = {0, 0, 127}));
  connect(esac6a1.EFD, gensal1.EFD) annotation(Line(points = {{22, 2}, {26, 2}, {26, 2.8}, {30, 2.8}}, color = {0, 0, 127}));
  connect(esac6a1.ETERM0, gensal1.ETERM0) annotation(Line(points = {{22, 4}, {26, 4}, {26, 5}, {29.8, 5}}, color = {0, 0, 127}));
  connect(esac6a1.EX_AUX, gensal1.EX_AUX) annotation(Line(points = {{22, 6.6}, {26, 6.6}, {26, 8}, {29.8, 8}}, color = {0, 0, 127}));
  connect(esac6a1.XADIFD, gensal1.XADIFD) annotation(Line(points = {{2, 13}, {-4, 13}, {-4, 24}, {56, 24}, {56, 5}, {50, 5}}, color = {0, 0, 127}));
  connect(const.y, esac6a1.VUEL) annotation(Line(points = {{-14.5, 16}, {-6, 16}, {-6, 10}, {2, 10}}, color = {0, 0, 127}));
  connect(esac6a1.VOEL, esac6a1.VUEL) annotation(Line(points = {{2, 7.2}, {-6, 7.2}, {-6, 10}, {2, 10}}, color = {0, 0, 127}));
  connect(esac6a1.VOTHSG, esac6a1.VUEL) annotation(Line(points = {{2, 4.4}, {-6, 4.4}, {-6, 10}, {2, 10}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.341686, -0.113895}, extent = {{-99.4305, 99.4305}, {99.4305, -99.4305}}), Text(origin = {-56.15, 42.48}, extent = {{138.84, -72.32}, {-24.72, 14.46}}, textString = "GENSAL_ESAC6A"), Text(origin = {67.6574, -76.3052}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "PIN")}));
end GENSAL_ESAC6A;

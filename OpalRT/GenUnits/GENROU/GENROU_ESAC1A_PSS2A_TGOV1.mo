within OpalRT.GenUnits.GENROU;
class GENROU_ESAC1A_PSS2A_TGOV1
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
  // ESAC1A Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real VAMAX_ex = 5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real VAMIN_ex = -5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KF_ex = 0.4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real TF_ex = 0.2 "(sec)" annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KD_ex = 0.4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real KE_ex = 0.5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real VRMAX_ex = 5 annotation(Dialog(tab = "ESAC1A Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "ESAC1A Parameters"));
  //
  // PSS2A Parameters
  parameter Real TW1_pss = 0.1 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW2_pss = 1 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T6_pss = 1 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW3_pss = 1.5 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real TW4_pss = 0.2 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T7_pss = 0.2 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS2_pss = 0.1 "T7/(2*H)" annotation(Dialog(tab = "PSS2A Parameters"));
  //T7/(2*H);
  parameter Real KS3_pss = 0.1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T8_pss = 0.05 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T9_pss = 0.01 ">0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real KS1_pss = 0.15 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T1_pss = 0.1 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T2_pss = 0.1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T3_pss = 0.01 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real T4_pss = 1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMAX_pss = 1 annotation(Dialog(tab = "PSS2A Parameters"));
  parameter Real VSTMIN_pss = -1 annotation(Dialog(tab = "PSS2A Parameters"));
  // PSS2A ICONs
  parameter Real M0_pss = 2 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M1_pss = 2 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M2_pss = 2 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M3_pss = 2 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M4_pss = 1 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  parameter Real M5_pss = 2 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2A Parameters", group = "ICONs"));
  //
  //
  // TGOV1 Parameters
  parameter Real R_tg = 0.06 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T1_tg = 0.5 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMAX_tg = 1 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMIN_tg = 0 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T2_tg = 1 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T3_tg = 1 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real Dt_tg = 0 annotation(Dialog(tab = "TGOV1 Parameters"));
  //****************************
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {15, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {22, -44}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = noVOEL) annotation(Placement(visible = true, transformation(origin = {-62, 9}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESAC1A esac1a1(TR = TR_ex, TB = TB_ex, TC = TC_ex, KA = KA_ex, TA = TA_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex) annotation(Placement(visible = true, transformation(origin = {-23, -3}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-74, -25}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-80, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {7, 10}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {20, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  OpalRT.Electrical.Control.Stabilizer.PSS2A pss2a1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-67, -9}, extent = {{-12.5, -8.33333}, {12.5, 8.33333}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-74, -41}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.TGOV1 tgov11(R = R_tg, T1 = T1_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, T2 = T2_tg, T3 = T3_tg, Dt = Dt_tg) annotation(Placement(visible = true, transformation(origin = {-21, -32}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = noVUEL) annotation(Placement(visible = true, transformation(origin = {-62, 25}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  parameter Real noVOEL(fixed = false, start = 1);
  parameter Real noVUEL(fixed = false, start = 1);
initial equation
  noVOEL = Modelica.Constants.inf;
  noVUEL = -Modelica.Constants.inf;
equation
  connect(genrou1.AccPower, pss2a1.PSS_AUX2[2]) annotation(Line(points = {{30, -18.9}, {42.6179, -18.9}, {42.6179, -56.9688}, {-83.0614, -56.9688}, {-83.0614, -14.5684}, {-79.5824, -14.5684}, {-79.5824, -14.5684}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, pss2a1.PSS_AUX2[1]) annotation(Line(points = {{30, -27}, {38.9214, -27}, {38.9214, -53.0549}, {-83.0614, -53.0549}, {-83.0614, -14.3509}, {-80.0173, -14.3509}, {-80.0173, -14.3509}}, color = {0, 0, 127}));
  connect(genrou1.VI, pss2a1.VI2) annotation(Line(points = {{30, -15}, {43.4876, -15}, {43.4876, -58.2735}, {-84.366, -58.2735}, {-84.366, -12.6114}, {-79.5824, -12.6114}, {-79.5824, -12.6114}}, color = {0, 0, 127}));
  connect(genrou1.VI, tgov11.VI) annotation(Line(points = {{30, -15}, {43.4876, -15}, {43.4876, -58.2735}, {-42.4005, -58.2735}, {-42.4005, -32.1809}, {-33.9204, -32.1809}, {-33.9204, -32.1809}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, tgov11.MBASE) annotation(Line(points = {{30, -23.1}, {41.0958, -23.1}, {41.0958, -55.6642}, {-40.8784, -55.6642}, {-40.8784, -37.1819}, {-32.3983, -37.1819}, {-32.3983, -37.1819}}, color = {0, 0, 127}));
  connect(pss2a1.VI2, pss2a1.VI) annotation(Line(points = {{-79.5, -12.3333}, {-84.366, -12.3333}, {-84.366, -7.3929}, {-80.0173, -7.3929}, {-80.0173, -7.3929}}, color = {0, 0, 127}));
  connect(dGREF, tgov11.dGREF) annotation(Line(points = {{-74, -41}, {-60.4478, -41}, {-60.4478, -22.6136}, {-33.9204, -22.6136}, {-33.9204, -22.6136}}));
  connect(genrou1.PMECH0, tgov11.PMECH0) annotation(Line(points = {{5.55112e-16, -27}, {-5.43596, -27}, {-5.43596, -24.788}, {-7.61034, -24.788}, {-7.61034, -24.788}}, color = {0, 0, 127}));
  connect(dVREF, esac1a1.dVREF) annotation(Line(points = {{-74, -25}, {-42.6179, -25}, {-42.6179, -12.6114}, {-38.704, -12.6114}, {-38.704, -12.6114}}));
  connect(esac1a1.VI, genrou1.VI) annotation(Line(points = {{-8, 6}, {38.704, 6}, {38.704, -15.0032}, {29.789, -15.0032}, {29.789, -15.0032}}, color = {0, 0, 127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{15, -30}, {15, -42}, {22, -42}, {22, -44}}));
  connect(esac1a1.EFD0, genrou1.EFD0) annotation(Line(points = {{-8, -15}, {-0.3, -15}}, color = {0, 0, 127}));
  connect(esac1a1.EFD, genrou1.EFD) annotation(Line(points = {{-8, -12}, {-4, -12}, {-4, -10.8}, {0, -10.8}}, color = {0, 0, 127}));
  connect(esac1a1.ETERM0, genrou1.ETERM0) annotation(Line(points = {{-8, -9}, {-4, -9}, {-4, -7.5}, {-0.3, -7.5}}, color = {0, 0, 127}));
  connect(esac1a1.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{-8, -5.1}, {-4, -5.1}, {-4, -3}, {-0.3, -3}}, color = {0, 0, 127}));
  connect(esac1a1.XADIFD, genrou1.XADIFD) annotation(Line(points = {{-38, 4.5}, {-46, 4.5}, {-46, 21}, {38, 21}, {38, -7.5}, {30, -7.5}}, color = {0, 0, 127}));
  connect(pss2a1.VOTHSG, esac1a1.VOTHSG) annotation(Line(points = {{-54.5, -14}, {-46, -14}, {-46, -8.4}, {-38, -8.4}}, color = {0, 0, 127}));
  connect(genrou1.TRIP, TRIP) annotation(Line(points = {{15, 0}, {15, 10}, {7, 10}}, color = {0, 0, 127}));
  connect(esac1a1.VOEL, const.y) annotation(Line(points = {{-38, -4.2}, {-53, -4.2}, {-53, 9}, {-56.5, 9}}, color = {0, 0, 127}));
  connect(constant1.y, esac1a1.VUEL) annotation(Line(points = {{-56.5, 25}, {-48, 25}, {-48, 0}, {-38, 0}}, color = {0, 0, 127}));
  connect(pss2a1.PSS_AUX2, pss2a1.PSS_AUX) annotation(Line(points = {{-79.25, -14}, {-83, -14}, {-83, -9}, {-79.5, -9}}, color = {0, 0, 127}));
  connect(tgov11.PMECH, genrou1.PMECH) annotation(Line(points = {{-8.5, -22}, {-4.75, -22}, {-4.75, -24}, {0, -24}}, color = {0, 0, 127}));
  connect(tgov11.SLIP, genrou1.SLIP) annotation(Line(points = {{-33.5, -42}, {-40, -42}, {-40, -53}, {39, -53}, {39, -27}, {30, -27}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={  Rectangle(origin = {1.13895, -0.455581}, extent = {{-95.8998, 87.016}, {94.0775, -92.2551}}), Text(origin = {81.3394, -65.6766}, extent = {{4.19912, -2.19976}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-17.8508, 8.14958}, extent = {{-73.46, 18.79}, {105.862, -43.6543}}, textString = "GENROU_ESAC1A_PSS2A_TGOV1"), Text(origin = {-45.8986, 48.5446}, extent = {{10.3467, 5.1464}, {-20.16, 17.89}}, textString = "dVREF"), Text(origin = {-3.67054, 68.6822}, extent = {{44.9869, -7.03202}, {3.03902, 10.3198}}, textString = "TRIP")}));
end GENROU_ESAC1A_PSS2A_TGOV1;

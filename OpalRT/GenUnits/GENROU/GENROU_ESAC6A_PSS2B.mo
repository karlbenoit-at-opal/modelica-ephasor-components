within OpalRT.GenUnits.GENROU;
class GENROU_ESAC6A_PSS2B
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
  // ESAC6A Parameters
  parameter Real TR_ex = 0.1 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KA_ex = 400 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TA_ex = 5 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TK_ex = 1 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TB_ex = 12 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TC_ex = 10 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VAMAX_ex = 5 "or zero" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VAMIN_ex = -5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VRMAX_ex = 5 "or zero" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VFELIM_ex = 5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KH_ex = 0.4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real VHMAX_ex = 5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TH_ex = 0.2 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real TJ_ex = 1.2 "(sec)" annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KC_ex = 0.4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KD_ex = 0.4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real KE_ex = 0.5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "ESAC6A Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "ESAC6A Parameters"));
  //
  // PSS2B Parameters
  parameter Real TW1_pss = 2 ">0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real TW2_pss = 2 "To bypass second washout, first signal: set Tw2 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T6_pss = 0.05 "To bypass first signal transducer: set T6 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real TW3_pss = 2 ">0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real TW4_pss = 1.5 "To bypass second washout, second signal: set Tw4 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T7_pss = 2 "To bypass second signal transducer: set T7 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real KS2_pss = 0.259 "T7/(2*H)" annotation(Dialog(tab = "PSS2B Parameters"));
  //T7/(2*H);
  parameter Real KS3_pss = 1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T8_pss = 0.5 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T9_pss = 0.1 ">0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real KS1_pss = 15 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T1_pss = 0.15 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T2_pss = 0.05 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T3_pss = 0.15 "To bypass first lead-lag: set T1 = T2 = 0" annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T4_pss = 0.05 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VSTMAX_pss = 0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VSTMIN_pss = -0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VS1MAX_pss = 0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VS1MIN_pss = -0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VS2MAX_pss = 0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real VS2MIN_pss = -0.1 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T10_pss = 0.3 annotation(Dialog(tab = "PSS2B Parameters"));
  parameter Real T11_pss = 0.15 ">0" annotation(Dialog(tab = "PSS2B Parameters"));
  /// PSS2B ICONs
  parameter Real M0_pss = 1 "ICS1, first stabilizer input code" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M1_pss = 0 "REMBUS1, first remote bus number" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M2_pss = 3 "ICS2, second stabilizer input code" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M3_pss = 0 "REMBUS2, second remote bus number" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M4_pss = 5 "M, ramp tracking filter" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  parameter Real M5_pss = 1 "N, ramp tracking filter" annotation(Dialog(tab = "PSS2B Parameters", group = "ICONs"));
  //-----------------------------------------------------
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {65, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {78, -40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESAC6A esac6a1(TR = TR_ex, KA = KA_ex, TA = TA_ex, TK = TK_ex, TB = TB_ex, TC = TC_ex, VAMAX = VAMAX_ex, VAMIN = VAMIN_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, TE = TE_ex, VFELIM = VFELIM_ex, KH = KH_ex, VHMAX = VHMAX_ex, TH = TH_ex, TJ = TJ_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {25, -3}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-30, -27}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-90, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {70, 10}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-15, 12}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2B pss2b1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = M_ID, T10 = T10_pss, T11 = T11_pss, VS1MAX = VS1MAX_pss, VS1MIN = VS1MIN_pss, VS2MAX = VS2MAX_pss, VS2MIN = VS2MIN_pss) annotation(Placement(visible = true, transformation(origin = {-24, -8}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
equation
  connect(genrou1.AccPower, pss2b1.PSS_AUX2[2]) annotation(Line(points = {{80, -18.9}, {86.323, -18.9}, {86.323, -46.3143}, {-43.9225, -46.3143}, {-43.9225, -14.3509}, {-39.3563, -14.3509}, {-39.3563, -14.3509}}, color = {0, 0, 127}));
  connect(genrou1.SLIP, pss2b1.PSS_AUX2[1]) annotation(Line(points = {{80, -27}, {86.323, -27}, {86.323, -46.3143}, {-43.9225, -46.3143}, {-43.9225, -14.1335}, {-39.3563, -14.1335}, {-39.3563, -14.1335}}, color = {0, 0, 127}));
  connect(pss2b1.VI2, pss2b1.VI) annotation(Line(points = {{-39, -12}, {-44.3574, -12}, {-44.3574, -5.87083}, {-39.5738, -5.87083}, {-39.5738, -5.87083}}, color = {0, 0, 127}));
  connect(genrou1.VI, pss2b1.VI) annotation(Line(points = {{80, -15}, {93.7159, -15}, {93.7159, 23.9182}, {-44.14, 23.9182}, {-44.14, -5.87083}, {-39.5738, -5.87083}, {-39.5738, -5.87083}}, color = {0, 0, 127}));
  connect(genrou1.VI, esac6a1.VI) annotation(Line(points = {{80, -15}, {93.7159, -15}, {93.7159, 6.08827}, {40.661, 6.08827}, {40.661, 6.08827}}, color = {0, 0, 127}));
  connect(dVREF, esac6a1.dVREF) annotation(Line(points = {{-30, -27}, {3.47901, -27}, {3.47901, -13.0463}, {10.2196, -13.0463}, {10.2196, -13.0463}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{70, 10}, {64.6465, 10}, {64.6465, 0}, {65, 0}}));
  connect(genrou1.p, bus0) annotation(Line(points = {{65, -30}, {64.6701, -30}, {64.6701, -39.6963}, {78, -39.6963}, {78, -40}}));
  connect(esac6a1.EFD0, genrou1.EFD0) annotation(Line(points = {{40, -15}, {49.7, -15}}, color = {0, 0, 127}));
  connect(esac6a1.EFD, genrou1.EFD) annotation(Line(points = {{40, -12}, {45, -12}, {45, -10.8}, {50, -10.8}}, color = {0, 0, 127}));
  connect(esac6a1.ETERM0, genrou1.ETERM0) annotation(Line(points = {{40, -9}, {45, -9}, {45, -7.5}, {49.7, -7.5}}, color = {0, 0, 127}));
  connect(esac6a1.EX_AUX, genrou1.EX_AUX) annotation(Line(points = {{40, -5.1}, {45, -5.1}, {45, -3}, {49.7, -3}}, color = {0, 0, 127}));
  connect(esac6a1.XADIFD, genrou1.XADIFD) annotation(Line(points = {{10, 4.5}, {5, 4.5}, {5, 22}, {85, 22}, {85, -7.5}, {80, -7.5}}, color = {0, 0, 127}));
  connect(esac6a1.VOTHSG, pss2b1.VOTHSG) annotation(Line(points = {{10, -8.4}, {0, -8.4}, {0, -14}, {-9, -14}}, color = {0, 0, 127}));
  connect(const.y, esac6a1.VOEL) annotation(Line(points = {{-9.5, 12}, {-6, 12}, {-6, -4.2}, {10, -4.2}}, color = {0, 0, 127}));
  connect(esac6a1.VUEL, esac6a1.VOEL) annotation(Line(points = {{10, 0}, {-6, 0}, {-6, -4.2}, {10, -4.2}}, color = {0, 0, 127}));
  connect(pss2b1.PSS_AUX2, pss2b1.PSS_AUX) annotation(Line(points = {{-38.7, -14}, {-44, -14}, {-44, -8}, {-39, -8}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation(Line(points = {{50, -27}, {45, -27}, {45, -24}, {50, -24}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={  Rectangle(origin = {1.13895, -0.455581}, extent = {{-95.8998, 87.016}, {94.0775, -92.2551}}), Text(origin = {59.2082, -48.8021}, extent = {{25.85, -23.58}, {-7.62995, 1.11752}}, textString = "PIN"), Text(origin = {3.79, 21.66}, extent = {{-90.22, -12.56}, {83.94, -29.04}}, textString = "GENROU_ESAC6A_PSS2B"), Text(origin = {-56.8041, -62.2026}, extent = {{32.7986, -14.6978}, {-20.16, 17.89}}, textString = "dVREF"), Text(origin = {-39.5506, 67.8396}, extent = {{59.06, -18.94}, {22.2309, 1.83495}}, textString = "TRIP"), Text(origin = {-57.7167, 43.7594}, extent = {{29.363, -8.03091}, {-20.16, 17.89}}, textString = "dGREF")}));
end GENROU_ESAC6A_PSS2B;

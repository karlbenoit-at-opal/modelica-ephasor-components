within OpalRT.GenUnits.GENROU;
class GENROU_IEEEX2_ST2CUT
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU"));
  parameter Real P_gen = 900 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 200 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU"));
  parameter Real Tdo_s = 0.3 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU"));
  parameter Real Tqo_p = 0.04 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU"));
  parameter Real Tqo_s = 0.4 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU"));
  parameter Real H = 2.6 "Inertia constant" annotation(Dialog(tab = "GENROU"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU"));
  parameter Real Xd = 0.67 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xq = 0.62 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xd_p = 0.3 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xq_p = 0.3 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xd_s = 0.01 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real Xl = 0.04 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU"));
  parameter Real S1 = 0.1 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU"));
  parameter Real S12 = 0.2 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU"));
  // IEEEX2 Parameters
  parameter Real TR_ex = 0.025 annotation(Dialog(tab = "IEEEX2"));
  parameter Real KA_ex = 98 annotation(Dialog(tab = "IEEEX2"));
  parameter Real TA_ex = 0.2 annotation(Dialog(tab = "IEEEX2"));
  parameter Real TB_ex = 0.1 annotation(Dialog(tab = "IEEEX2"));
  parameter Real TC_ex = 0.2 annotation(Dialog(tab = "IEEEX2"));
  parameter Real VRMAX_ex = 9 annotation(Dialog(tab = "IEEEX2"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "IEEEX2"));
  parameter Real KE_ex = 0.5 annotation(Dialog(tab = "IEEEX2"));
  parameter Real TE_ex = 0.35 annotation(Dialog(tab = "IEEEX2"));
  parameter Real KF_ex = 0.03 annotation(Dialog(tab = "IEEEX2"));
  parameter Real TF1_ex = 0.4 annotation(Dialog(tab = "IEEEX2"));
  parameter Real TF2_ex = 0.4 annotation(Dialog(tab = "IEEEX2"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "IEEEX2"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "IEEEX2"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "IEEEX2"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "IEEEX2"));
  // ST2CUT
  //parameter Integer IBUS = 100 "Located Bus No." annotation(Dialog(tab = "ST2CUT"));
  //parameter String ID = M_ID "Located Bus No." annotation(Dialog(tab = "ST2CUT"));
  parameter Real K1_pss = 1 annotation(Dialog(tab = "ST2CUT"));
  parameter Real K2_pss = 1 annotation(Dialog(tab = "ST2CUT"));
  parameter Real T1_pss = 0.1 "sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real T2_pss = 0.1 "sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real T3_pss = 0.1 "T3>0 sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real T4_pss = 0.1 "T4>0 sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real T5_pss = 0.1 "sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real T6_pss = 0.1 "sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real T7_pss = 0.1 "sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real T8_pss = 0.1 "sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real T9_pss = 0.1 "sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real T10_pss = 0.1 "sec" annotation(Dialog(tab = "ST2CUT"));
  parameter Real LSMAX_pss = 0.3 annotation(Dialog(tab = "ST2CUT"));
  parameter Real LSMIN_pss = -0.3 annotation(Dialog(tab = "ST2CUT"));
  parameter Real VCU_pss = 1.2 "VCU (pu) (if equal zero, ignored)" annotation(Dialog(tab = "ST2CUT"));
  parameter Real VCL_pss = -0.1 "VCL (pu) (if equal zero, ignored)" annotation(Dialog(tab = "ST2CUT"));
  // ICONS
  parameter Real M0_pss = 1 "ICS1, first stabilizer input code" annotation(Dialog(tab = "ST2CUT"));
  parameter Real M1_pss = 2 "IB1, first remote bus number. CURRENLY DISABLED" annotation(Dialog(tab = "ST2CUT"));
  parameter Real M2_pss = 3 "ICS2, second stabilizer input code" annotation(Dialog(tab = "ST2CUT"));
  parameter Real M3_pss = 0 "B2, second remote bus number CURRENLY DISABLED" annotation(Dialog(tab = "ST2CUT"));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-150, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEEX2 ieeex21(TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, TF2 = TF2_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin = {-68, 28}, extent = {{-30, -30}, {30, 30}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, -60}, extent = {{-11.25, -11.25}, {11.25, 11.25}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-31, -31}, {31, 31}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-38, 78}, extent = {{-12, -12}, {12, 12}}, rotation = 0), iconTransformation(origin = {-196, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.ST2CUT st2cut1(K1 = K1_pss, K2 = K2_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, T7 = T7_pss, T8 = T8_pss, T9 = T9_pss, T10 = T10_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss, M2 = M2_pss, M3 = M3_pss) annotation(Placement(visible = true, transformation(origin = {-160, 40}, extent = {{-27, -27}, {27, 27}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-200, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-196, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ieeex21.dVREF, dVREF) annotation(Line(points={{-98,10},{-124.422,10},
          {-124.422,0.514139},{-200,0.514139},{-200,0}},                                                                                                color = {0, 0, 127}));
  connect(st2cut1.PSS_AUX2, st2cut1.PSS_AUX) annotation(Line(points={{-187,23.8},
          {-193.83,23.8},{-193.83,40.3599},{-187,40.3599},{-187,40}},                                                                                                color = {0, 0, 127}));
  connect(genrou1.SLIP, st2cut1.PSS_AUX[1]) annotation(Line(points={{51,-24.8},
          {87.4036,-24.8},{87.4036,-76.0925},{-193.83,-76.0925},{-193.83,
          40.3599},{-187,40.3599},{-187,38.65}},                                                                                                                                                              color = {0, 0, 127}));
  connect(genrou1.AccPower, st2cut1.PSS_AUX[2]) annotation(Line(points={{51,
          -8.06},{87.4036,-8.06},{87.4036,-76.0925},{-193.83,-76.0925},{-193.83,
          40.3599},{-187,40.3599},{-187,41.35}},                                                                                                                                                                  color = {0, 0, 127}));
  connect(st2cut1.VI2, st2cut1.VI) annotation(Line(points={{-187,29.2},{
          -196.658,29.2},{-196.658,44.9871},{-187,44.9871},{-187,45.4}},                                                                                     color = {0, 0, 127}));
  connect(genrou1.VI, st2cut1.VI) annotation(Line(points={{51,0},{94.8586,0},{
          94.8586,-80.4627},{-196.658,-80.4627},{-196.658,44.9871},{-187,
          44.9871},{-187,45.4}},                                                                                                                                                              color = {0, 0, 127}));
  connect(st2cut1.VOTHSG, ieeex21.VOTHSG) annotation(Line(points={{-133,23.8},{
          -124.883,23.8},{-124.883,16.9014},{-98,16.9014},{-98,17.2}},                                                                                            color = {0, 0, 127}));
  connect(genrou1.XADIFD, ieeex21.XADIFD) annotation(Line(points={{51,15.5},{
          68.0751,15.5},{68.0751,94.3662},{-107.981,94.3662},{-107.981,42.2535},
          {-98,42.2535},{-98,43}},                                                                                                                                                                color = {0, 0, 127}));
  connect(genrou1.VI, ieeex21.VI) annotation(Line(points={{51,0},{94.8586,0},{
          94.8586,46.5296},{-38,46.5296},{-38,46}},                                                                                               color = {0, 0, 127}));
  connect(ieeex21.VOEL, const.y) annotation(Line(points={{-98,25.6},{-112.207,
          25.6},{-112.207,84.0376},{-139,84.0376},{-139,84}},                                                                                             color = {0, 0, 127}));
  connect(const.y, ieeex21.VUEL) annotation(Line(points={{-139,84},{-112.207,84},
          {-112.207,33.8028},{-98,33.8028},{-98,34}},                                                                                          color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points={{-38,78},{21.1268,78},{
          21.1268,31},{20,31}}));
  connect(genrou1.p, bus0) annotation(Line(points={{20,-31},{20.1878,-31},{
          20.1878,-59.1549},{60,-59.1549},{60,-60}}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation(Line(points={{-11,-24.8},{
          -21.1268,-24.8},{-21.1268,-16.9014},{-12.6761,-16.9014},{-12.6761,
          -18.6},{-11,-18.6}},                                                                                                                                                               color = {0, 0, 127}));
  connect(genrou1.EFD0, ieeex21.EFD0) annotation(Line(points={{-11.62,0},{
          -29.5775,0},{-29.5775,4.22535},{-38,4.22535},{-38,4}},                                                                                            color = {0, 0, 127}));
  connect(ieeex21.EFD, genrou1.EFD) annotation(Line(points={{-38,10},{-10.7981,
          10},{-10.7981,8.68},{-11,8.68}},                                                                                          color = {0, 0, 127}));
  connect(genrou1.ETERM0, ieeex21.ETERM0) annotation(Line(points={{-11.62,15.5},
          {-37.0892,15.5},{-37.0892,16},{-38,16}},                                                                                               color = {0, 0, 127}));
  connect(genrou1.EX_AUX, ieeex21.EX_AUX) annotation(Line(points={{-11.62,24.8},
          {-36.1502,24.8},{-36.1502,23.8},{-38,23.8}},                                                                                           color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-110.567, 36.1123}, extent = {{-72.35, 23.36}, {183.461, -71.5985}}, textString = "GENROU_IEEEX2_ST2CUT"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN"), Rectangle(origin = {-49.0602, 0}, extent = {{-148.452, 97.9886}, {146.761, -98.1985}}), Text(origin = {-165.133, 74.8658}, extent = {{-26.73, 13.27}, {24.2521, -8.31425}}, textString = "TRIP"), Text(origin = {-161.238, -66.37}, extent = {{-22.12, 13.63}, {22.12, -13.63}}, textString = "dVREF")}));
end GENROU_IEEEX2_ST2CUT;

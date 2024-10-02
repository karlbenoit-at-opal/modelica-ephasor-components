within OpalRT.GenUnits.GENROU;
block GENROU_SCRX_PSS2A_GAST_UEL2
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
  // SCRX Parameters
  parameter Real TA_TB_ex = 4 "TA/TB" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TB_ex = 1 "(>0) (sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real K_ex = 100 annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TE_ex = 0.5 "(sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMIN_ex = -1.2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMAX_ex = 2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real CSWITCH_ex = 1 "0 for bus fed, 1 for solid fed" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real rc_rfd_ex = 3 "rc/rfd, 0 with negative field current capability (EX=EFD)" annotation(Dialog(tab = "SCRX Parameters"));
  // PSS2A Parameters
  parameter String PSS_ID = M_ID "Machine Identifier" annotation(Dialog(tab = "PSS2A Parameters"));
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
  // GAST Parameters
  parameter Real R_tg = 0.01 "Speed droop" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T1_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T2_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T3_tg = 0.3 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real AT_tg = 0.12 "Ambient temperature load limit" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real KT_tg = 0.2 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMAX_tg = 0.12 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMIN_tg = 0.01 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real DTURB_tg = 0.01 annotation(Dialog(tab = "GAST Parameters"));
  // UEL2 parameters
  parameter Real TUV_uel = 5 "(sec) voltage filter time constant" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TUP_uel = 5 "(sec) real power filter time constant" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TUQ_uel = 0 "(sec) reactive power filter time constant" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real KUI_uel = 0.5 "(pu) UEL integral gain" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real KUL_uel = 0.8 "(pu) UEL proportional gain" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real VUIMAX_uel = 0.25 "(pu) UEL integrator output maximum limit" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real VUIMIN_uel = 0 "(pu) UEL integrator output minimum limit" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real KUF_uel = 0 "(pu) UEL excitation system stabilizer gain" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real KFB_uel = 0 "(pu)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TUL_uel = 0 "(sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TU1_uel = 0 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TU2_uel = 0 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TU3_uel = 0 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real TU4_uel = 0 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P0_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q0_uel = -0.31 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P1_uel = 0.3 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q1_uel = -0.31 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P2_uel = 0.6 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q2_uel = -0.28 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P3_uel = 0.9 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q3_uel = -0.21 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P4_uel = 1.02 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q4_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P5_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q5_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P6_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q6_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P7_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q7_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P8_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q8_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P9_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q9_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real P10_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real Q10_uel = 0 "(pu on gen. MVA base)" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real VULMAX_uel = 0.25 "(pu) UEL output maximum limit" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real VULMIN_uel = 0 "(pu) UEL output minimum limit" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real M0_uel = 0 "k1, exponent in function F1" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real M1_uel = 0 "k2, exponent in function F2" annotation(Dialog(tab = "UEL2 Parameters"));
  parameter Real M2_uel = 0 " 0: MVAR curve interpreted as mirror image around MVAR axis; 1: MVAR is found by linear extrapolation" annotation(Dialog(tab = "UEL2 Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {60, -20}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={71.5,-54.5}, extent={{-3.5, -3.5},{3.5,3.5}}, rotation = 0), iconTransformation(origin={100,-81},extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-58,-30},  extent={{-5,-5},{5,5}}, rotation = 0), iconTransformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={51,12},    extent={{-4,-4},{4,4}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.SCRX scrx1(IBUS = IBUS, ID = M_ID, TA_TB = TA_TB_ex, TB = TB_ex, K = K_ex, TE = TE_ex, EMIN = EMIN_ex, EMAX = EMAX_ex, CSWITCH = CSWITCH_ex, rc_rfd = rc_rfd_ex) annotation(Placement(visible = true, transformation(origin={-1.5,0.5}, extent={{-25.5,
            -25.5},{25.5,25.5}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.GAST gast1(R = R_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, AT = AT_tg, KT = KT_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, DTURB = DTURB_tg, IBUS = IBUS, ID = M_ID) annotation(Placement(visible = true, transformation(origin={3,-49}, extent={{-18,-18},
            {18,18}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.PSS2A pSS2A1(M2 = M2_pss, TW1 = TW1_pss, TW2 = TW2_pss, T6 = T6_pss, TW3 = TW3_pss, TW4 = TW4_pss, T7 = T7_pss, KS2 = KS2_pss, KS3 = KS3_pss, T8 = T8_pss, T9 = T9_pss, KS1 = KS1_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, VSTMAX = VSTMAX_pss, VSTMIN = VSTMIN_pss, M0 = M0_pss, M1 = M1_pss, M3 = M3_pss, M4 = M4_pss, M5 = M5_pss, initType = Modelica.Blocks.Types.Init.InitialOutput, ID = PSS_ID) annotation(Placement(visible = true, transformation(origin={-65,-9}, extent = {{-15, -10}, {15, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-41.5,
            -1.5}, extent = {{-3.5,
            -3.5},{3.5,3.5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin={-32.5,-34.5}, extent={{-4.5,-4.5},{4.5,4.5}}, rotation = 0), iconTransformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.UnderExcitationLimiter.UEL2 uel21(TUV = TUV_uel, TUP = TUP_uel, TUQ = TUQ_uel, KUI = KUI_uel, KUL = KUL_uel, VUIMAX = VUIMAX_uel, VUIMIN = VUIMIN_uel, KUF = KUF_uel, KFB = KFB_uel, TUL = TUL_uel, TU1 = TU1_uel, TU2 = TU2_uel, TU3 = TU3_uel, TU4 = TU4_uel, P0 = P0_uel, Q0 = Q0_uel, P1 = P1_uel, Q1 = Q1_uel, P2 = P2_uel, Q2 = Q2_uel, P3 = P3_uel, Q3 = Q3_uel, P4 = P4_uel, Q4 = Q4_uel, P5 = P5_uel, Q5 = Q5_uel, P6 = P6_uel, Q6 = Q6_uel, P7 = P7_uel, Q7 = Q7_uel, P8 = P8_uel, Q8 = Q8_uel, P9 = P9_uel, Q9 = Q9_uel, P10 = P10_uel, Q10 = Q10_uel, VULMAX = VULMAX_uel, VULMIN = VULMIN_uel, M0 = M0_uel, M1 = M1_uel, M2 = M2_uel) annotation(Placement(visible = true, transformation(origin={-84.5,30}, extent = {{-15, -15}, {37.5, 15}}, rotation = 0)));
equation 
  // connect(uel21.VFB, uel21.VF) annotation(Line(points = {{-105, -52}, {-106.383, -52}, {-92.5, -46.0993}, {-92.5, -46}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points={{51,12},{60.0473,12},{60.0473,
          5},{60,5}}));
  connect(bus0, genrou1.p) annotation(Line(points={{71.5,-54.5},{60,-54.5},{60,-45}}));
  connect(scrx1.EFD0, genrou1.EFD0) annotation (Line(points = {{24,-19.9},{29,
          -19.9},{29,-20},{34.5,-20}}, color = {0,0,127}));
  connect(scrx1.EFD, genrou1.EFD) annotation (Line(points = {{24,-14.8},{28.5,
          -14.8},{28.5,-13},{35,-13}}, color = {0,0,127}));
  connect(scrx1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{24,-9.7},{
          29,-9.7},{29,-7.5},{34.5,-7.5}}, color = {0,0,127}));
  connect(scrx1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{24,-3.07},{
          29.5,-3.07},{29.5,0},{34.5,0}}, color = {0,0,127}));
  connect(gast1.PMECH, genrou1.PMECH) annotation (Line(points = {{21,-34.6},{
          26.5,-34.6},{26.5,-35},{35,-35}}, color = {0,0,127}));
  connect(genrou1.PMECH0, gast1.PMECH0) annotation (Line(points={{35,-40},{30,-40},
          {30,-38.2},{21,-38.2}}, color = {0,0,127}));
  connect(scrx1.XADIFD, genrou1.XADIFD) annotation (Line(points={{-27,13.25},{-33,
          13.25},{-33,30},{90,30},{90,-7.5},{85,-7.5}}, color = {0,0,127}));
  connect(pSS2A1.VOTHSG, scrx1.VOTHSG) annotation (Line(points = {{-50,-15},{
          -50,-8.68},{-27,-8.68}},
                               color = {0,0,127}));
  connect(scrx1.VUEL, uel21.VUEL) annotation (Line(points={{-27,5.6},{-43,5.6},{
          -43,30},{-45.95,30}}, color = {0,0,127}));
  connect(const.y, scrx1.VOEL) annotation (Line(points = {{-37.65,-1.5},{
          -35.325,-1.5},{-35.325,-1.54},{-27,-1.54}}, color = {0,0,127}));
  connect(genrou1.SLIP, gast1.SLIP) annotation (Line(points={{85,-40},{88,-40},{
          88,-70},{-20,-70},{-20,-63.4},{-15,-63.4}}, color = {0,0,127}));
  connect(uel21.VF, scrx1.VF) annotation (Line(points={{-98.975,30},{-117,30},{-117,
          55},{34,55},{34,20.9},{25.02,20.9}}, color = {0,0,127}));
  connect(uel21.EX_AUX, genrou1.EX_AUX) annotation (Line(points={{-98.975,42},{-131,
          42},{-131,69},{30,69},{30,0},{34.5,0}}, color = {0,0,127}));
  connect(genrou1.MBASE, gast1.MBASE) annotation (Line(points={{85,-33.5},{90,-33.5},
          {90,-73},{-23,-73},{-23,-56.2},{-15,-56.2}}, color={0,0,127}));
  connect(genrou1.VI, gast1.VI) annotation (Line(points={{85,-20},{94,-20},{94,-77},
          {-27,-77},{-27,-49},{-15,-49}}, color={0,0,127}));
  connect(dGREF, gast1.dGREF) annotation (Line(points={{-32.5,-34.5},{-24.25,-34.5},
          {-24.25,-34.6},{-15,-34.6}}, color={0,0,0}));
  connect(genrou1.VI, scrx1.VI) annotation (Line(points={{85,-20},{94,-20},{94,34},
          {37,34},{37,15.8},{24,15.8}}, color={0,0,127}));
  connect(genrou1.VI, uel21.VI) annotation (Line(points={{85,-20},{94,-20},{94,50},
          {-106,50},{-106,36},{-98.975,36}}, color={0,0,127}));
  connect(dVREF, scrx1.dVREF) annotation (Line(points={{-58,-30},{-40,-30},{-40,
          -14.8},{-27,-14.8}}, color={0,0,0}));
  connect(genrou1.VI, pSS2A1.VI) annotation (Line(points={{85,-20},{94,-20},{94,
          -77},{-86,-77},{-86,-7},{-80,-7}}, color={0,0,127}));
  connect(genrou1.VI, pSS2A1.VI2) annotation (Line(points={{85,-20},{94,-20},{94,
          -77},{-86,-77},{-86,-13},{-80,-13}}, color={0,0,127}));
  connect(genrou1.SLIP, pSS2A1.PSS_AUX[1]) annotation (Line(points={{85,-40},{88,
          -40},{88,-70},{-84,-70},{-84,-9.5},{-80,-9.5}}, color={0,0,127}));
  connect(pSS2A1.PSS_AUX2, pSS2A1.PSS_AUX) annotation (Line(points={{-80,-15},{-84,
          -15},{-84,-9},{-80,-9}}, color={0,0,127}));
  connect(genrou1.AccPower, pSS2A1.PSS_AUX[2]) annotation (Line(points={{85,-26.5},
          {97,-26.5},{97,-80},{-89,-80},{-89,-8.5},{-80,-8.5}}, color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={ Rectangle(origin = {0, -0.141443}, extent = {{-99.5757, 100}, {99.5757, -100}}), Text(origin = {-41.567, 8.40626}, extent = {{-47.035, 9.06456}, {131.12, -33.38}}, textString = "GENROU_SCRX_PSS2A_GAST_UEL2"), Text(origin = {-42.9469, -55.5302}, extent = {{99.1458, -14.4242}, {131.12, -33.38}}, textString = "PIN"), Text(origin = {-186.3, 103.48}, extent = {{99.15, -14.42}, {131.12, -33.38}}, textString = "TRIP"), Text(origin = {-185.687, -54.538}, extent = {{99.15, -14.42}, {143.177, -35.2713}}, textString = "dVREF"), Text(origin = {-186.56, -24.0692}, extent = {{99.15, -14.42}, {143.18, -35.27}}, textString = "dGREF")}));
end GENROU_SCRX_PSS2A_GAST_UEL2;

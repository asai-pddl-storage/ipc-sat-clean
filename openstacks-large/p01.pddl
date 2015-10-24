(define (problem os-sequencedstrips-p170_1)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) 
(stacks-avail n0)

(waiting o1)
(includes o1 p20)(includes o1 p22)(includes o1 p50)(includes o1 p55)(includes o1 p72)(includes o1 p76)(includes o1 p93)(includes o1 p102)(includes o1 p136)(includes o1 p139)(includes o1 p142)

(waiting o2)
(includes o2 p59)(includes o2 p112)(includes o2 p163)

(waiting o3)
(includes o3 p4)(includes o3 p27)(includes o3 p30)(includes o3 p36)(includes o3 p41)(includes o3 p91)(includes o3 p113)(includes o3 p155)

(waiting o4)
(includes o4 p19)(includes o4 p26)(includes o4 p41)(includes o4 p94)(includes o4 p133)(includes o4 p138)(includes o4 p155)

(waiting o5)
(includes o5 p22)(includes o5 p28)(includes o5 p74)(includes o5 p83)(includes o5 p84)(includes o5 p116)

(waiting o6)
(includes o6 p29)(includes o6 p154)

(waiting o7)
(includes o7 p16)(includes o7 p21)(includes o7 p139)(includes o7 p143)(includes o7 p160)

(waiting o8)
(includes o8 p24)(includes o8 p98)(includes o8 p131)

(waiting o9)
(includes o9 p36)(includes o9 p71)(includes o9 p80)

(waiting o10)
(includes o10 p74)(includes o10 p78)(includes o10 p157)(includes o10 p161)

(waiting o11)
(includes o11 p17)(includes o11 p24)(includes o11 p39)(includes o11 p78)(includes o11 p148)(includes o11 p163)

(waiting o12)
(includes o12 p19)(includes o12 p48)(includes o12 p101)

(waiting o13)
(includes o13 p27)(includes o13 p69)(includes o13 p129)(includes o13 p134)(includes o13 p154)

(waiting o14)
(includes o14 p83)(includes o14 p85)

(waiting o15)
(includes o15 p28)(includes o15 p31)(includes o15 p32)(includes o15 p102)(includes o15 p122)

(waiting o16)
(includes o16 p57)(includes o16 p151)

(waiting o17)
(includes o17 p89)(includes o17 p103)(includes o17 p152)(includes o17 p157)

(waiting o18)
(includes o18 p3)(includes o18 p39)(includes o18 p46)(includes o18 p53)(includes o18 p77)

(waiting o19)
(includes o19 p17)(includes o19 p59)(includes o19 p63)(includes o19 p68)(includes o19 p78)(includes o19 p108)

(waiting o20)
(includes o20 p2)(includes o20 p11)(includes o20 p12)(includes o20 p13)(includes o20 p54)(includes o20 p60)(includes o20 p134)

(waiting o21)
(includes o21 p72)(includes o21 p100)(includes o21 p131)(includes o21 p137)(includes o21 p141)

(waiting o22)
(includes o22 p83)(includes o22 p136)(includes o22 p154)

(waiting o23)
(includes o23 p20)(includes o23 p53)(includes o23 p114)(includes o23 p147)(includes o23 p163)(includes o23 p169)

(waiting o24)
(includes o24 p4)(includes o24 p50)(includes o24 p56)(includes o24 p72)(includes o24 p76)(includes o24 p118)(includes o24 p130)(includes o24 p151)(includes o24 p169)

(waiting o25)
(includes o25 p95)(includes o25 p97)(includes o25 p99)(includes o25 p106)(includes o25 p140)(includes o25 p167)

(waiting o26)
(includes o26 p67)(includes o26 p73)(includes o26 p103)(includes o26 p162)

(waiting o27)
(includes o27 p31)(includes o27 p89)(includes o27 p133)

(waiting o28)
(includes o28 p46)(includes o28 p70)(includes o28 p97)(includes o28 p103)(includes o28 p116)(includes o28 p130)

(waiting o29)
(includes o29 p13)(includes o29 p29)(includes o29 p48)(includes o29 p61)(includes o29 p95)(includes o29 p101)

(waiting o30)
(includes o30 p10)(includes o30 p16)(includes o30 p25)(includes o30 p56)(includes o30 p72)(includes o30 p76)(includes o30 p81)(includes o30 p165)

(waiting o31)
(includes o31 p23)(includes o31 p44)(includes o31 p47)(includes o31 p160)

(waiting o32)
(includes o32 p28)(includes o32 p31)(includes o32 p73)(includes o32 p116)(includes o32 p122)(includes o32 p138)

(waiting o33)
(includes o33 p20)(includes o33 p36)

(waiting o34)
(includes o34 p40)(includes o34 p43)(includes o34 p56)(includes o34 p63)(includes o34 p65)(includes o34 p86)(includes o34 p165)

(waiting o35)
(includes o35 p26)(includes o35 p49)(includes o35 p51)(includes o35 p98)(includes o35 p112)(includes o35 p142)

(waiting o36)
(includes o36 p22)(includes o36 p122)(includes o36 p125)(includes o36 p150)(includes o36 p163)

(waiting o37)
(includes o37 p105)(includes o37 p130)(includes o37 p135)(includes o37 p166)

(waiting o38)
(includes o38 p25)(includes o38 p30)(includes o38 p147)(includes o38 p168)

(waiting o39)
(includes o39 p13)(includes o39 p29)(includes o39 p59)(includes o39 p87)

(waiting o40)
(includes o40 p35)(includes o40 p42)(includes o40 p53)(includes o40 p67)(includes o40 p81)(includes o40 p97)(includes o40 p107)(includes o40 p127)(includes o40 p145)(includes o40 p147)(includes o40 p169)

(waiting o41)
(includes o41 p59)(includes o41 p124)

(waiting o42)
(includes o42 p24)(includes o42 p37)(includes o42 p76)(includes o42 p97)(includes o42 p143)(includes o42 p149)

(waiting o43)
(includes o43 p14)(includes o43 p17)(includes o43 p59)(includes o43 p62)(includes o43 p71)(includes o43 p161)(includes o43 p162)

(waiting o44)
(includes o44 p46)(includes o44 p89)(includes o44 p122)(includes o44 p130)(includes o44 p136)(includes o44 p142)(includes o44 p155)

(waiting o45)
(includes o45 p46)(includes o45 p50)(includes o45 p59)(includes o45 p97)(includes o45 p126)(includes o45 p135)

(waiting o46)
(includes o46 p24)(includes o46 p45)(includes o46 p49)(includes o46 p153)

(waiting o47)
(includes o47 p57)(includes o47 p143)

(waiting o48)
(includes o48 p130)(includes o48 p152)

(waiting o49)
(includes o49 p17)(includes o49 p24)(includes o49 p53)(includes o49 p95)(includes o49 p98)(includes o49 p119)(includes o49 p165)

(waiting o50)
(includes o50 p14)(includes o50 p30)(includes o50 p38)(includes o50 p59)(includes o50 p66)(includes o50 p79)(includes o50 p83)(includes o50 p104)(includes o50 p149)

(waiting o51)
(includes o51 p16)(includes o51 p82)(includes o51 p104)(includes o51 p137)

(waiting o52)
(includes o52 p130)(includes o52 p148)(includes o52 p163)

(waiting o53)
(includes o53 p27)(includes o53 p92)

(waiting o54)
(includes o54 p46)(includes o54 p49)(includes o54 p106)(includes o54 p146)(includes o54 p151)(includes o54 p166)

(waiting o55)
(includes o55 p22)(includes o55 p31)(includes o55 p122)(includes o55 p128)(includes o55 p142)

(waiting o56)
(includes o56 p36)(includes o56 p87)

(waiting o57)
(includes o57 p54)(includes o57 p58)(includes o57 p63)(includes o57 p79)(includes o57 p86)

(waiting o58)
(includes o58 p16)(includes o58 p70)(includes o58 p81)(includes o58 p85)(includes o58 p115)(includes o58 p165)

(waiting o59)
(includes o59 p23)(includes o59 p24)(includes o59 p30)(includes o59 p36)(includes o59 p96)(includes o59 p124)

(waiting o60)
(includes o60 p31)(includes o60 p36)(includes o60 p57)(includes o60 p112)(includes o60 p157)

(waiting o61)
(includes o61 p43)(includes o61 p58)(includes o61 p162)

(waiting o62)
(includes o62 p22)(includes o62 p35)(includes o62 p50)(includes o62 p92)(includes o62 p98)(includes o62 p131)(includes o62 p150)

(waiting o63)
(includes o63 p16)(includes o63 p47)(includes o63 p50)(includes o63 p53)(includes o63 p77)(includes o63 p91)

(waiting o64)
(includes o64 p26)(includes o64 p32)(includes o64 p74)(includes o64 p113)(includes o64 p133)

(waiting o65)
(includes o65 p166)

(waiting o66)
(includes o66 p21)(includes o66 p86)

(waiting o67)
(includes o67 p1)(includes o67 p18)(includes o67 p60)(includes o67 p80)(includes o67 p108)(includes o67 p155)

(waiting o68)
(includes o68 p55)(includes o68 p95)(includes o68 p106)

(waiting o69)
(includes o69 p47)(includes o69 p61)(includes o69 p67)

(waiting o70)
(includes o70 p28)(includes o70 p93)(includes o70 p100)(includes o70 p136)(includes o70 p152)(includes o70 p163)

(waiting o71)
(includes o71 p10)(includes o71 p14)(includes o71 p15)(includes o71 p20)(includes o71 p30)(includes o71 p120)(includes o71 p128)

(waiting o72)
(includes o72 p78)(includes o72 p102)(includes o72 p105)(includes o72 p152)

(waiting o73)
(includes o73 p21)(includes o73 p59)(includes o73 p73)(includes o73 p85)(includes o73 p103)(includes o73 p110)(includes o73 p137)

(waiting o74)
(includes o74 p20)(includes o74 p22)(includes o74 p55)(includes o74 p73)(includes o74 p120)(includes o74 p150)

(waiting o75)
(includes o75 p16)(includes o75 p63)

(waiting o76)
(includes o76 p62)(includes o76 p63)(includes o76 p123)(includes o76 p143)(includes o76 p162)

(waiting o77)
(includes o77 p54)(includes o77 p65)(includes o77 p116)(includes o77 p124)(includes o77 p134)(includes o77 p158)

(waiting o78)
(includes o78 p70)(includes o78 p115)(includes o78 p122)

(waiting o79)
(includes o79 p14)(includes o79 p43)(includes o79 p118)

(waiting o80)
(includes o80 p6)(includes o80 p9)(includes o80 p10)(includes o80 p15)(includes o80 p52)(includes o80 p61)(includes o80 p68)(includes o80 p80)(includes o80 p85)(includes o80 p114)

(waiting o81)
(includes o81 p35)(includes o81 p75)(includes o81 p77)(includes o81 p81)(includes o81 p115)(includes o81 p125)

(waiting o82)
(includes o82 p33)(includes o82 p65)(includes o82 p68)(includes o82 p117)(includes o82 p159)

(waiting o83)
(includes o83 p3)(includes o83 p6)(includes o83 p17)(includes o83 p19)(includes o83 p56)

(waiting o84)
(includes o84 p18)(includes o84 p59)(includes o84 p98)(includes o84 p132)(includes o84 p147)

(waiting o85)
(includes o85 p38)(includes o85 p61)(includes o85 p62)(includes o85 p169)

(waiting o86)
(includes o86 p53)(includes o86 p115)(includes o86 p116)

(waiting o87)
(includes o87 p76)

(waiting o88)
(includes o88 p24)(includes o88 p36)(includes o88 p87)(includes o88 p98)(includes o88 p120)(includes o88 p125)

(waiting o89)
(includes o89 p71)(includes o89 p101)(includes o89 p119)(includes o89 p145)

(waiting o90)
(includes o90 p39)(includes o90 p67)(includes o90 p100)(includes o90 p114)(includes o90 p128)(includes o90 p129)(includes o90 p133)(includes o90 p140)

(waiting o91)
(includes o91 p41)(includes o91 p71)(includes o91 p88)(includes o91 p123)(includes o91 p156)

(waiting o92)
(includes o92 p13)(includes o92 p55)(includes o92 p80)(includes o92 p146)

(waiting o93)
(includes o93 p30)(includes o93 p34)(includes o93 p38)(includes o93 p40)(includes o93 p88)(includes o93 p106)(includes o93 p154)

(waiting o94)
(includes o94 p118)(includes o94 p129)(includes o94 p151)

(waiting o95)
(includes o95 p22)(includes o95 p68)(includes o95 p96)(includes o95 p116)(includes o95 p135)

(waiting o96)
(includes o96 p11)(includes o96 p82)(includes o96 p91)(includes o96 p154)(includes o96 p163)(includes o96 p170)

(waiting o97)
(includes o97 p6)(includes o97 p22)(includes o97 p83)(includes o97 p88)(includes o97 p101)(includes o97 p102)

(waiting o98)
(includes o98 p9)(includes o98 p37)(includes o98 p152)

(waiting o99)
(includes o99 p36)(includes o99 p40)(includes o99 p44)(includes o99 p75)(includes o99 p112)(includes o99 p124)(includes o99 p130)

(waiting o100)
(includes o100 p27)(includes o100 p95)(includes o100 p110)

(waiting o101)
(includes o101 p10)(includes o101 p71)(includes o101 p83)(includes o101 p101)(includes o101 p118)(includes o101 p127)(includes o101 p169)

(waiting o102)
(includes o102 p16)(includes o102 p91)(includes o102 p167)

(waiting o103)
(includes o103 p124)

(waiting o104)
(includes o104 p14)(includes o104 p30)(includes o104 p65)(includes o104 p129)(includes o104 p141)

(waiting o105)
(includes o105 p14)(includes o105 p62)(includes o105 p84)(includes o105 p91)(includes o105 p105)(includes o105 p116)(includes o105 p126)(includes o105 p131)(includes o105 p161)(includes o105 p166)

(waiting o106)
(includes o106 p17)(includes o106 p37)(includes o106 p77)(includes o106 p89)(includes o106 p126)(includes o106 p127)(includes o106 p146)

(waiting o107)
(includes o107 p17)(includes o107 p74)(includes o107 p105)(includes o107 p119)

(waiting o108)
(includes o108 p81)(includes o108 p164)

(waiting o109)
(includes o109 p13)(includes o109 p20)(includes o109 p32)(includes o109 p49)(includes o109 p84)(includes o109 p103)(includes o109 p145)(includes o109 p148)

(waiting o110)
(includes o110 p26)(includes o110 p29)(includes o110 p51)(includes o110 p100)(includes o110 p137)

(waiting o111)
(includes o111 p57)(includes o111 p60)(includes o111 p63)(includes o111 p123)(includes o111 p140)

(waiting o112)
(includes o112 p2)(includes o112 p33)(includes o112 p84)(includes o112 p99)(includes o112 p107)(includes o112 p168)

(waiting o113)
(includes o113 p34)(includes o113 p79)(includes o113 p120)(includes o113 p140)

(waiting o114)
(includes o114 p5)(includes o114 p13)(includes o114 p48)(includes o114 p60)(includes o114 p82)(includes o114 p131)(includes o114 p161)(includes o114 p168)(includes o114 p170)

(waiting o115)
(includes o115 p42)(includes o115 p63)(includes o115 p117)(includes o115 p124)(includes o115 p138)(includes o115 p144)

(waiting o116)
(includes o116 p27)(includes o116 p58)(includes o116 p79)(includes o116 p143)(includes o116 p154)(includes o116 p169)

(waiting o117)
(includes o117 p4)(includes o117 p28)(includes o117 p37)(includes o117 p53)(includes o117 p57)(includes o117 p90)(includes o117 p93)(includes o117 p96)(includes o117 p104)(includes o117 p116)(includes o117 p125)(includes o117 p164)

(waiting o118)
(includes o118 p15)(includes o118 p59)(includes o118 p65)(includes o118 p144)(includes o118 p154)(includes o118 p155)

(waiting o119)
(includes o119 p3)(includes o119 p82)(includes o119 p85)(includes o119 p104)(includes o119 p158)

(waiting o120)
(includes o120 p22)(includes o120 p28)(includes o120 p33)(includes o120 p51)(includes o120 p116)(includes o120 p150)

(waiting o121)
(includes o121 p9)(includes o121 p26)(includes o121 p41)(includes o121 p56)(includes o121 p62)(includes o121 p69)(includes o121 p76)(includes o121 p111)(includes o121 p165)

(waiting o122)
(includes o122 p15)(includes o122 p67)(includes o122 p149)(includes o122 p166)

(waiting o123)
(includes o123 p81)(includes o123 p115)(includes o123 p133)(includes o123 p153)(includes o123 p161)

(waiting o124)
(includes o124 p39)(includes o124 p51)(includes o124 p55)(includes o124 p73)(includes o124 p93)(includes o124 p96)(includes o124 p133)

(waiting o125)
(includes o125 p62)(includes o125 p119)(includes o125 p149)

(waiting o126)
(includes o126 p4)(includes o126 p5)(includes o126 p15)(includes o126 p51)(includes o126 p107)(includes o126 p110)(includes o126 p139)

(waiting o127)
(includes o127 p6)(includes o127 p60)(includes o127 p138)(includes o127 p156)

(waiting o128)
(includes o128 p68)

(waiting o129)
(includes o129 p7)(includes o129 p17)(includes o129 p25)(includes o129 p80)(includes o129 p83)(includes o129 p150)

(waiting o130)
(includes o130 p32)(includes o130 p45)(includes o130 p112)(includes o130 p148)(includes o130 p165)

(waiting o131)
(includes o131 p11)(includes o131 p34)(includes o131 p36)(includes o131 p130)(includes o131 p135)(includes o131 p142)(includes o131 p154)(includes o131 p168)

(waiting o132)
(includes o132 p23)(includes o132 p58)(includes o132 p98)(includes o132 p102)(includes o132 p125)(includes o132 p165)

(waiting o133)
(includes o133 p40)(includes o133 p42)(includes o133 p46)(includes o133 p47)(includes o133 p53)(includes o133 p85)(includes o133 p105)(includes o133 p109)(includes o133 p133)(includes o133 p144)

(waiting o134)
(includes o134 p16)(includes o134 p31)(includes o134 p155)

(waiting o135)
(includes o135 p30)(includes o135 p167)

(waiting o136)
(includes o136 p22)(includes o136 p26)(includes o136 p74)(includes o136 p106)(includes o136 p116)(includes o136 p157)

(waiting o137)
(includes o137 p26)(includes o137 p35)(includes o137 p38)(includes o137 p113)

(waiting o138)
(includes o138 p8)(includes o138 p19)(includes o138 p26)(includes o138 p59)(includes o138 p61)(includes o138 p71)(includes o138 p156)

(waiting o139)
(includes o139 p8)(includes o139 p79)

(waiting o140)
(includes o140 p69)(includes o140 p95)(includes o140 p119)(includes o140 p134)(includes o140 p154)(includes o140 p161)(includes o140 p170)

(waiting o141)
(includes o141 p90)(includes o141 p93)(includes o141 p142)

(waiting o142)
(includes o142 p2)

(waiting o143)
(includes o143 p50)(includes o143 p150)(includes o143 p163)

(waiting o144)
(includes o144 p17)(includes o144 p63)(includes o144 p76)(includes o144 p80)(includes o144 p118)(includes o144 p155)

(waiting o145)
(includes o145 p15)(includes o145 p100)(includes o145 p153)

(waiting o146)
(includes o146 p12)

(waiting o147)
(includes o147 p61)(includes o147 p98)(includes o147 p100)

(waiting o148)
(includes o148 p6)(includes o148 p50)(includes o148 p79)(includes o148 p110)(includes o148 p120)(includes o148 p127)(includes o148 p144)(includes o148 p159)(includes o148 p161)(includes o148 p162)

(waiting o149)
(includes o149 p10)(includes o149 p16)(includes o149 p21)(includes o149 p29)(includes o149 p54)(includes o149 p68)(includes o149 p85)(includes o149 p99)(includes o149 p108)(includes o149 p113)(includes o149 p137)

(waiting o150)
(includes o150 p78)(includes o150 p89)(includes o150 p137)

(waiting o151)
(includes o151 p82)(includes o151 p86)(includes o151 p95)(includes o151 p139)(includes o151 p163)

(waiting o152)
(includes o152 p15)(includes o152 p46)(includes o152 p49)(includes o152 p58)(includes o152 p61)(includes o152 p68)(includes o152 p84)(includes o152 p98)(includes o152 p142)

(waiting o153)
(includes o153 p40)(includes o153 p57)(includes o153 p116)(includes o153 p128)

(waiting o154)
(includes o154 p34)(includes o154 p57)(includes o154 p85)(includes o154 p101)(includes o154 p121)

(waiting o155)
(includes o155 p5)(includes o155 p42)(includes o155 p53)(includes o155 p74)(includes o155 p120)(includes o155 p144)(includes o155 p169)

(waiting o156)
(includes o156 p76)(includes o156 p99)(includes o156 p127)

(waiting o157)
(includes o157 p28)(includes o157 p35)(includes o157 p58)(includes o157 p72)(includes o157 p84)(includes o157 p112)(includes o157 p114)(includes o157 p122)(includes o157 p152)

(waiting o158)
(includes o158 p22)(includes o158 p35)(includes o158 p103)

(waiting o159)
(includes o159 p78)(includes o159 p104)(includes o159 p127)(includes o159 p146)(includes o159 p157)

(waiting o160)
(includes o160 p3)(includes o160 p113)(includes o160 p114)

(waiting o161)
(includes o161 p102)

(waiting o162)
(includes o162 p17)(includes o162 p92)(includes o162 p118)(includes o162 p132)

(waiting o163)
(includes o163 p5)(includes o163 p18)(includes o163 p43)(includes o163 p58)(includes o163 p69)(includes o163 p72)(includes o163 p169)

(waiting o164)
(includes o164 p3)(includes o164 p19)(includes o164 p106)(includes o164 p120)(includes o164 p149)

(waiting o165)
(includes o165 p9)(includes o165 p54)(includes o165 p76)(includes o165 p98)

(waiting o166)
(includes o166 p17)(includes o166 p30)(includes o166 p64)(includes o166 p82)(includes o166 p85)(includes o166 p107)

(waiting o167)
(includes o167 p51)(includes o167 p98)(includes o167 p99)

(waiting o168)
(includes o168 p46)(includes o168 p60)(includes o168 p138)

(waiting o169)
(includes o169 p19)(includes o169 p131)(includes o169 p147)

(waiting o170)
(includes o170 p89)(includes o170 p116)(includes o170 p150)

(not-made p1)
(not-made p2)
(not-made p3)
(not-made p4)
(not-made p5)
(not-made p6)
(not-made p7)
(not-made p8)
(not-made p9)
(not-made p10)
(not-made p11)
(not-made p12)
(not-made p13)
(not-made p14)
(not-made p15)
(not-made p16)
(not-made p17)
(not-made p18)
(not-made p19)
(not-made p20)
(not-made p21)
(not-made p22)
(not-made p23)
(not-made p24)
(not-made p25)
(not-made p26)
(not-made p27)
(not-made p28)
(not-made p29)
(not-made p30)
(not-made p31)
(not-made p32)
(not-made p33)
(not-made p34)
(not-made p35)
(not-made p36)
(not-made p37)
(not-made p38)
(not-made p39)
(not-made p40)
(not-made p41)
(not-made p42)
(not-made p43)
(not-made p44)
(not-made p45)
(not-made p46)
(not-made p47)
(not-made p48)
(not-made p49)
(not-made p50)
(not-made p51)
(not-made p52)
(not-made p53)
(not-made p54)
(not-made p55)
(not-made p56)
(not-made p57)
(not-made p58)
(not-made p59)
(not-made p60)
(not-made p61)
(not-made p62)
(not-made p63)
(not-made p64)
(not-made p65)
(not-made p66)
(not-made p67)
(not-made p68)
(not-made p69)
(not-made p70)
(not-made p71)
(not-made p72)
(not-made p73)
(not-made p74)
(not-made p75)
(not-made p76)
(not-made p77)
(not-made p78)
(not-made p79)
(not-made p80)
(not-made p81)
(not-made p82)
(not-made p83)
(not-made p84)
(not-made p85)
(not-made p86)
(not-made p87)
(not-made p88)
(not-made p89)
(not-made p90)
(not-made p91)
(not-made p92)
(not-made p93)
(not-made p94)
(not-made p95)
(not-made p96)
(not-made p97)
(not-made p98)
(not-made p99)
(not-made p100)
(not-made p101)
(not-made p102)
(not-made p103)
(not-made p104)
(not-made p105)
(not-made p106)
(not-made p107)
(not-made p108)
(not-made p109)
(not-made p110)
(not-made p111)
(not-made p112)
(not-made p113)
(not-made p114)
(not-made p115)
(not-made p116)
(not-made p117)
(not-made p118)
(not-made p119)
(not-made p120)
(not-made p121)
(not-made p122)
(not-made p123)
(not-made p124)
(not-made p125)
(not-made p126)
(not-made p127)
(not-made p128)
(not-made p129)
(not-made p130)
(not-made p131)
(not-made p132)
(not-made p133)
(not-made p134)
(not-made p135)
(not-made p136)
(not-made p137)
(not-made p138)
(not-made p139)
(not-made p140)
(not-made p141)
(not-made p142)
(not-made p143)
(not-made p144)
(not-made p145)
(not-made p146)
(not-made p147)
(not-made p148)
(not-made p149)
(not-made p150)
(not-made p151)
(not-made p152)
(not-made p153)
(not-made p154)
(not-made p155)
(not-made p156)
(not-made p157)
(not-made p158)
(not-made p159)
(not-made p160)
(not-made p161)
(not-made p162)
(not-made p163)
(not-made p164)
(not-made p165)
(not-made p166)
(not-made p167)
(not-made p168)
(not-made p169)
(not-made p170)

(= (total-cost) 0)

)

(:goal
(and
(shipped o1)
(shipped o2)
(shipped o3)
(shipped o4)
(shipped o5)
(shipped o6)
(shipped o7)
(shipped o8)
(shipped o9)
(shipped o10)
(shipped o11)
(shipped o12)
(shipped o13)
(shipped o14)
(shipped o15)
(shipped o16)
(shipped o17)
(shipped o18)
(shipped o19)
(shipped o20)
(shipped o21)
(shipped o22)
(shipped o23)
(shipped o24)
(shipped o25)
(shipped o26)
(shipped o27)
(shipped o28)
(shipped o29)
(shipped o30)
(shipped o31)
(shipped o32)
(shipped o33)
(shipped o34)
(shipped o35)
(shipped o36)
(shipped o37)
(shipped o38)
(shipped o39)
(shipped o40)
(shipped o41)
(shipped o42)
(shipped o43)
(shipped o44)
(shipped o45)
(shipped o46)
(shipped o47)
(shipped o48)
(shipped o49)
(shipped o50)
(shipped o51)
(shipped o52)
(shipped o53)
(shipped o54)
(shipped o55)
(shipped o56)
(shipped o57)
(shipped o58)
(shipped o59)
(shipped o60)
(shipped o61)
(shipped o62)
(shipped o63)
(shipped o64)
(shipped o65)
(shipped o66)
(shipped o67)
(shipped o68)
(shipped o69)
(shipped o70)
(shipped o71)
(shipped o72)
(shipped o73)
(shipped o74)
(shipped o75)
(shipped o76)
(shipped o77)
(shipped o78)
(shipped o79)
(shipped o80)
(shipped o81)
(shipped o82)
(shipped o83)
(shipped o84)
(shipped o85)
(shipped o86)
(shipped o87)
(shipped o88)
(shipped o89)
(shipped o90)
(shipped o91)
(shipped o92)
(shipped o93)
(shipped o94)
(shipped o95)
(shipped o96)
(shipped o97)
(shipped o98)
(shipped o99)
(shipped o100)
(shipped o101)
(shipped o102)
(shipped o103)
(shipped o104)
(shipped o105)
(shipped o106)
(shipped o107)
(shipped o108)
(shipped o109)
(shipped o110)
(shipped o111)
(shipped o112)
(shipped o113)
(shipped o114)
(shipped o115)
(shipped o116)
(shipped o117)
(shipped o118)
(shipped o119)
(shipped o120)
(shipped o121)
(shipped o122)
(shipped o123)
(shipped o124)
(shipped o125)
(shipped o126)
(shipped o127)
(shipped o128)
(shipped o129)
(shipped o130)
(shipped o131)
(shipped o132)
(shipped o133)
(shipped o134)
(shipped o135)
(shipped o136)
(shipped o137)
(shipped o138)
(shipped o139)
(shipped o140)
(shipped o141)
(shipped o142)
(shipped o143)
(shipped o144)
(shipped o145)
(shipped o146)
(shipped o147)
(shipped o148)
(shipped o149)
(shipped o150)
(shipped o151)
(shipped o152)
(shipped o153)
(shipped o154)
(shipped o155)
(shipped o156)
(shipped o157)
(shipped o158)
(shipped o159)
(shipped o160)
(shipped o161)
(shipped o162)
(shipped o163)
(shipped o164)
(shipped o165)
(shipped o166)
(shipped o167)
(shipped o168)
(shipped o169)
(shipped o170)
))

(:metric minimize (total-cost))

)


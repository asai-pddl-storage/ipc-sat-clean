(define (problem os-sequencedstrips-p190_2)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) 
(stacks-avail n0)

(waiting o1)
(includes o1 p70)(includes o1 p78)(includes o1 p113)(includes o1 p144)(includes o1 p170)

(waiting o2)
(includes o2 p19)(includes o2 p28)(includes o2 p94)(includes o2 p123)(includes o2 p132)

(waiting o3)
(includes o3 p71)(includes o3 p80)(includes o3 p148)

(waiting o4)
(includes o4 p35)

(waiting o5)
(includes o5 p56)(includes o5 p96)(includes o5 p97)(includes o5 p105)(includes o5 p108)(includes o5 p122)(includes o5 p170)

(waiting o6)
(includes o6 p37)(includes o6 p41)(includes o6 p58)(includes o6 p62)(includes o6 p81)(includes o6 p159)(includes o6 p160)(includes o6 p189)

(waiting o7)
(includes o7 p48)(includes o7 p152)

(waiting o8)
(includes o8 p30)(includes o8 p34)(includes o8 p39)(includes o8 p42)(includes o8 p112)

(waiting o9)
(includes o9 p23)(includes o9 p37)(includes o9 p75)(includes o9 p88)(includes o9 p95)(includes o9 p166)

(waiting o10)
(includes o10 p50)(includes o10 p59)(includes o10 p72)(includes o10 p98)(includes o10 p105)(includes o10 p134)(includes o10 p144)

(waiting o11)
(includes o11 p14)(includes o11 p71)(includes o11 p95)(includes o11 p115)(includes o11 p130)

(waiting o12)
(includes o12 p10)(includes o12 p26)(includes o12 p37)(includes o12 p51)(includes o12 p84)(includes o12 p97)(includes o12 p131)(includes o12 p133)(includes o12 p151)

(waiting o13)
(includes o13 p27)(includes o13 p56)(includes o13 p98)(includes o13 p156)(includes o13 p190)

(waiting o14)
(includes o14 p57)(includes o14 p62)(includes o14 p88)(includes o14 p178)

(waiting o15)
(includes o15 p101)(includes o15 p118)(includes o15 p164)(includes o15 p169)(includes o15 p175)(includes o15 p179)

(waiting o16)
(includes o16 p20)(includes o16 p39)(includes o16 p155)(includes o16 p173)(includes o16 p177)

(waiting o17)
(includes o17 p5)(includes o17 p28)(includes o17 p29)(includes o17 p31)(includes o17 p52)(includes o17 p60)(includes o17 p90)(includes o17 p92)(includes o17 p104)(includes o17 p130)(includes o17 p133)(includes o17 p137)(includes o17 p147)

(waiting o18)
(includes o18 p37)(includes o18 p60)(includes o18 p103)(includes o18 p115)(includes o18 p124)(includes o18 p141)(includes o18 p144)

(waiting o19)
(includes o19 p66)(includes o19 p189)

(waiting o20)
(includes o20 p46)(includes o20 p112)(includes o20 p130)(includes o20 p134)(includes o20 p146)(includes o20 p158)(includes o20 p163)

(waiting o21)
(includes o21 p85)(includes o21 p125)

(waiting o22)
(includes o22 p31)(includes o22 p36)(includes o22 p78)(includes o22 p144)(includes o22 p149)(includes o22 p155)

(waiting o23)
(includes o23 p32)(includes o23 p53)(includes o23 p161)

(waiting o24)
(includes o24 p12)(includes o24 p46)(includes o24 p61)(includes o24 p87)(includes o24 p106)(includes o24 p153)(includes o24 p169)(includes o24 p176)

(waiting o25)
(includes o25 p37)(includes o25 p40)(includes o25 p60)(includes o25 p109)(includes o25 p117)(includes o25 p122)(includes o25 p124)(includes o25 p125)(includes o25 p138)(includes o25 p171)

(waiting o26)
(includes o26 p17)(includes o26 p29)(includes o26 p31)(includes o26 p74)(includes o26 p134)(includes o26 p138)

(waiting o27)
(includes o27 p14)(includes o27 p139)(includes o27 p164)(includes o27 p183)

(waiting o28)
(includes o28 p12)(includes o28 p25)(includes o28 p58)(includes o28 p71)(includes o28 p98)(includes o28 p157)(includes o28 p177)(includes o28 p180)

(waiting o29)
(includes o29 p25)(includes o29 p50)(includes o29 p75)(includes o29 p101)(includes o29 p119)(includes o29 p174)

(waiting o30)
(includes o30 p8)(includes o30 p22)(includes o30 p90)(includes o30 p92)(includes o30 p153)(includes o30 p180)

(waiting o31)
(includes o31 p39)(includes o31 p43)(includes o31 p167)

(waiting o32)
(includes o32 p6)(includes o32 p45)(includes o32 p47)(includes o32 p83)(includes o32 p87)(includes o32 p90)(includes o32 p104)

(waiting o33)
(includes o33 p23)(includes o33 p82)(includes o33 p100)(includes o33 p188)

(waiting o34)
(includes o34 p107)(includes o34 p181)

(waiting o35)
(includes o35 p36)(includes o35 p73)(includes o35 p100)(includes o35 p161)(includes o35 p173)

(waiting o36)
(includes o36 p55)(includes o36 p61)(includes o36 p86)(includes o36 p105)(includes o36 p106)(includes o36 p148)(includes o36 p153)(includes o36 p154)

(waiting o37)
(includes o37 p66)(includes o37 p86)(includes o37 p139)(includes o37 p185)

(waiting o38)
(includes o38 p1)(includes o38 p14)(includes o38 p66)

(waiting o39)
(includes o39 p4)(includes o39 p78)(includes o39 p163)(includes o39 p181)(includes o39 p184)

(waiting o40)
(includes o40 p4)(includes o40 p50)(includes o40 p89)(includes o40 p103)(includes o40 p151)(includes o40 p156)(includes o40 p159)(includes o40 p160)(includes o40 p175)

(waiting o41)
(includes o41 p21)

(waiting o42)
(includes o42 p23)(includes o42 p24)(includes o42 p178)

(waiting o43)
(includes o43 p62)(includes o43 p84)(includes o43 p117)

(waiting o44)
(includes o44 p83)(includes o44 p122)(includes o44 p176)

(waiting o45)
(includes o45 p6)(includes o45 p83)(includes o45 p90)(includes o45 p111)(includes o45 p131)

(waiting o46)
(includes o46 p28)(includes o46 p76)(includes o46 p108)(includes o46 p111)(includes o46 p112)

(waiting o47)
(includes o47 p152)

(waiting o48)
(includes o48 p2)(includes o48 p42)(includes o48 p68)(includes o48 p84)(includes o48 p97)(includes o48 p130)(includes o48 p140)

(waiting o49)
(includes o49 p33)(includes o49 p44)(includes o49 p51)(includes o49 p100)(includes o49 p113)(includes o49 p131)(includes o49 p183)

(waiting o50)
(includes o50 p50)(includes o50 p61)(includes o50 p120)(includes o50 p124)(includes o50 p190)

(waiting o51)
(includes o51 p39)(includes o51 p45)(includes o51 p58)(includes o51 p63)(includes o51 p84)

(waiting o52)
(includes o52 p70)(includes o52 p106)(includes o52 p129)

(waiting o53)
(includes o53 p42)(includes o53 p92)(includes o53 p115)(includes o53 p135)(includes o53 p137)(includes o53 p139)(includes o53 p170)

(waiting o54)
(includes o54 p19)(includes o54 p87)(includes o54 p106)(includes o54 p107)(includes o54 p123)(includes o54 p169)

(waiting o55)
(includes o55 p9)(includes o55 p18)(includes o55 p32)(includes o55 p43)(includes o55 p59)(includes o55 p132)(includes o55 p153)(includes o55 p169)(includes o55 p181)(includes o55 p183)

(waiting o56)
(includes o56 p44)(includes o56 p99)(includes o56 p138)(includes o56 p162)(includes o56 p165)

(waiting o57)
(includes o57 p12)(includes o57 p121)

(waiting o58)
(includes o58 p9)(includes o58 p53)(includes o58 p96)(includes o58 p149)

(waiting o59)
(includes o59 p76)(includes o59 p156)

(waiting o60)
(includes o60 p10)(includes o60 p55)(includes o60 p102)

(waiting o61)
(includes o61 p54)(includes o61 p139)

(waiting o62)
(includes o62 p3)(includes o62 p6)(includes o62 p59)(includes o62 p87)(includes o62 p98)(includes o62 p146)(includes o62 p172)

(waiting o63)
(includes o63 p26)(includes o63 p58)(includes o63 p91)(includes o63 p94)(includes o63 p99)(includes o63 p102)(includes o63 p117)(includes o63 p182)

(waiting o64)
(includes o64 p107)

(waiting o65)
(includes o65 p68)(includes o65 p95)(includes o65 p124)(includes o65 p136)(includes o65 p163)(includes o65 p167)(includes o65 p176)(includes o65 p178)(includes o65 p189)

(waiting o66)
(includes o66 p12)(includes o66 p25)(includes o66 p47)(includes o66 p111)(includes o66 p140)(includes o66 p155)

(waiting o67)
(includes o67 p2)(includes o67 p3)(includes o67 p6)(includes o67 p7)(includes o67 p19)(includes o67 p175)

(waiting o68)
(includes o68 p67)(includes o68 p73)(includes o68 p100)(includes o68 p130)(includes o68 p180)(includes o68 p181)

(waiting o69)
(includes o69 p88)(includes o69 p125)

(waiting o70)
(includes o70 p1)(includes o70 p17)(includes o70 p119)

(waiting o71)
(includes o71 p51)(includes o71 p68)(includes o71 p105)(includes o71 p147)(includes o71 p174)(includes o71 p181)

(waiting o72)
(includes o72 p4)(includes o72 p42)(includes o72 p124)(includes o72 p131)(includes o72 p135)(includes o72 p148)(includes o72 p187)

(waiting o73)
(includes o73 p69)(includes o73 p75)(includes o73 p152)

(waiting o74)
(includes o74 p17)(includes o74 p60)(includes o74 p102)(includes o74 p120)(includes o74 p131)(includes o74 p162)(includes o74 p187)

(waiting o75)
(includes o75 p101)

(waiting o76)
(includes o76 p38)(includes o76 p73)(includes o76 p83)(includes o76 p94)

(waiting o77)
(includes o77 p48)(includes o77 p72)(includes o77 p156)(includes o77 p159)(includes o77 p171)(includes o77 p184)

(waiting o78)
(includes o78 p9)(includes o78 p48)(includes o78 p73)(includes o78 p171)

(waiting o79)
(includes o79 p18)(includes o79 p169)

(waiting o80)
(includes o80 p3)(includes o80 p117)(includes o80 p136)

(waiting o81)
(includes o81 p40)(includes o81 p85)(includes o81 p95)(includes o81 p110)(includes o81 p126)(includes o81 p145)(includes o81 p147)(includes o81 p149)

(waiting o82)
(includes o82 p13)(includes o82 p17)(includes o82 p33)(includes o82 p51)(includes o82 p92)(includes o82 p106)(includes o82 p117)(includes o82 p146)(includes o82 p177)

(waiting o83)
(includes o83 p63)(includes o83 p80)(includes o83 p118)(includes o83 p159)

(waiting o84)
(includes o84 p52)(includes o84 p74)(includes o84 p138)(includes o84 p153)

(waiting o85)
(includes o85 p2)(includes o85 p108)(includes o85 p133)

(waiting o86)
(includes o86 p74)(includes o86 p82)(includes o86 p117)(includes o86 p186)

(waiting o87)
(includes o87 p18)(includes o87 p49)(includes o87 p121)(includes o87 p158)(includes o87 p168)

(waiting o88)
(includes o88 p79)(includes o88 p101)(includes o88 p153)(includes o88 p168)

(waiting o89)
(includes o89 p15)(includes o89 p28)(includes o89 p76)(includes o89 p101)(includes o89 p102)(includes o89 p143)(includes o89 p159)

(waiting o90)
(includes o90 p44)(includes o90 p51)(includes o90 p90)(includes o90 p100)(includes o90 p109)

(waiting o91)
(includes o91 p8)(includes o91 p48)(includes o91 p79)(includes o91 p115)(includes o91 p125)(includes o91 p126)(includes o91 p176)

(waiting o92)
(includes o92 p39)(includes o92 p122)(includes o92 p154)(includes o92 p177)

(waiting o93)
(includes o93 p46)(includes o93 p48)(includes o93 p50)(includes o93 p91)(includes o93 p98)(includes o93 p118)(includes o93 p123)(includes o93 p170)(includes o93 p190)

(waiting o94)
(includes o94 p22)(includes o94 p58)(includes o94 p66)(includes o94 p70)(includes o94 p86)(includes o94 p124)(includes o94 p156)(includes o94 p182)(includes o94 p184)(includes o94 p187)

(waiting o95)
(includes o95 p9)(includes o95 p24)(includes o95 p52)(includes o95 p70)(includes o95 p74)(includes o95 p87)(includes o95 p88)(includes o95 p143)(includes o95 p184)

(waiting o96)
(includes o96 p52)(includes o96 p92)(includes o96 p127)

(waiting o97)
(includes o97 p106)(includes o97 p108)(includes o97 p140)(includes o97 p166)

(waiting o98)
(includes o98 p33)(includes o98 p85)(includes o98 p113)(includes o98 p114)(includes o98 p158)(includes o98 p174)(includes o98 p177)

(waiting o99)
(includes o99 p28)(includes o99 p157)

(waiting o100)
(includes o100 p9)(includes o100 p13)(includes o100 p57)(includes o100 p119)(includes o100 p139)

(waiting o101)
(includes o101 p54)(includes o101 p64)(includes o101 p93)(includes o101 p99)(includes o101 p106)(includes o101 p118)(includes o101 p129)(includes o101 p161)

(waiting o102)
(includes o102 p57)(includes o102 p165)

(waiting o103)
(includes o103 p4)(includes o103 p8)(includes o103 p64)(includes o103 p136)(includes o103 p180)

(waiting o104)
(includes o104 p23)(includes o104 p68)(includes o104 p121)(includes o104 p131)

(waiting o105)
(includes o105 p47)(includes o105 p81)

(waiting o106)
(includes o106 p91)(includes o106 p130)(includes o106 p150)(includes o106 p165)

(waiting o107)
(includes o107 p120)(includes o107 p159)

(waiting o108)
(includes o108 p4)(includes o108 p44)(includes o108 p102)(includes o108 p172)

(waiting o109)
(includes o109 p25)(includes o109 p47)(includes o109 p175)

(waiting o110)
(includes o110 p21)(includes o110 p47)(includes o110 p54)(includes o110 p93)(includes o110 p106)(includes o110 p129)(includes o110 p130)(includes o110 p143)(includes o110 p163)(includes o110 p171)

(waiting o111)
(includes o111 p88)(includes o111 p172)(includes o111 p173)

(waiting o112)
(includes o112 p11)(includes o112 p57)(includes o112 p141)(includes o112 p155)

(waiting o113)
(includes o113 p47)(includes o113 p87)(includes o113 p96)(includes o113 p101)(includes o113 p117)(includes o113 p128)(includes o113 p129)(includes o113 p146)(includes o113 p149)(includes o113 p157)

(waiting o114)
(includes o114 p21)(includes o114 p47)(includes o114 p53)(includes o114 p121)(includes o114 p141)

(waiting o115)
(includes o115 p6)(includes o115 p84)(includes o115 p125)(includes o115 p177)

(waiting o116)
(includes o116 p21)(includes o116 p27)(includes o116 p48)(includes o116 p111)(includes o116 p160)

(waiting o117)
(includes o117 p5)(includes o117 p50)(includes o117 p56)(includes o117 p98)(includes o117 p103)(includes o117 p107)(includes o117 p112)(includes o117 p155)(includes o117 p178)

(waiting o118)
(includes o118 p8)(includes o118 p119)(includes o118 p126)

(waiting o119)
(includes o119 p11)(includes o119 p50)(includes o119 p83)(includes o119 p97)(includes o119 p109)(includes o119 p140)

(waiting o120)
(includes o120 p29)(includes o120 p39)(includes o120 p140)(includes o120 p177)

(waiting o121)
(includes o121 p20)(includes o121 p130)

(waiting o122)
(includes o122 p8)(includes o122 p27)(includes o122 p62)(includes o122 p91)(includes o122 p111)(includes o122 p116)(includes o122 p186)

(waiting o123)
(includes o123 p7)(includes o123 p45)(includes o123 p86)(includes o123 p140)(includes o123 p177)(includes o123 p182)

(waiting o124)
(includes o124 p10)(includes o124 p52)(includes o124 p58)

(waiting o125)
(includes o125 p27)(includes o125 p99)(includes o125 p117)

(waiting o126)
(includes o126 p6)(includes o126 p34)(includes o126 p36)(includes o126 p56)(includes o126 p57)(includes o126 p68)(includes o126 p111)(includes o126 p116)(includes o126 p165)(includes o126 p175)

(waiting o127)
(includes o127 p68)

(waiting o128)
(includes o128 p59)(includes o128 p75)(includes o128 p165)(includes o128 p174)

(waiting o129)
(includes o129 p7)(includes o129 p28)(includes o129 p34)(includes o129 p45)

(waiting o130)
(includes o130 p20)(includes o130 p28)(includes o130 p91)(includes o130 p104)(includes o130 p164)

(waiting o131)
(includes o131 p182)

(waiting o132)
(includes o132 p20)(includes o132 p26)(includes o132 p44)(includes o132 p69)(includes o132 p86)(includes o132 p100)(includes o132 p111)(includes o132 p125)(includes o132 p149)(includes o132 p158)(includes o132 p163)(includes o132 p176)(includes o132 p183)

(waiting o133)
(includes o133 p49)(includes o133 p106)(includes o133 p164)(includes o133 p170)

(waiting o134)
(includes o134 p6)(includes o134 p26)(includes o134 p29)(includes o134 p33)(includes o134 p41)(includes o134 p94)(includes o134 p120)(includes o134 p138)(includes o134 p144)(includes o134 p155)

(waiting o135)
(includes o135 p26)(includes o135 p50)(includes o135 p65)(includes o135 p78)(includes o135 p92)(includes o135 p108)(includes o135 p111)(includes o135 p162)

(waiting o136)
(includes o136 p6)(includes o136 p9)(includes o136 p15)(includes o136 p28)(includes o136 p72)(includes o136 p77)(includes o136 p88)(includes o136 p155)(includes o136 p164)

(waiting o137)
(includes o137 p35)(includes o137 p141)(includes o137 p149)

(waiting o138)
(includes o138 p64)(includes o138 p76)(includes o138 p143)(includes o138 p189)

(waiting o139)
(includes o139 p58)(includes o139 p86)(includes o139 p125)(includes o139 p169)(includes o139 p189)

(waiting o140)
(includes o140 p29)(includes o140 p47)(includes o140 p84)(includes o140 p91)(includes o140 p142)(includes o140 p158)

(waiting o141)
(includes o141 p4)(includes o141 p74)(includes o141 p128)(includes o141 p151)(includes o141 p153)(includes o141 p155)(includes o141 p156)(includes o141 p184)

(waiting o142)
(includes o142 p32)(includes o142 p38)(includes o142 p61)(includes o142 p91)(includes o142 p106)(includes o142 p111)(includes o142 p124)(includes o142 p133)(includes o142 p164)(includes o142 p166)

(waiting o143)
(includes o143 p27)(includes o143 p38)(includes o143 p124)(includes o143 p148)

(waiting o144)
(includes o144 p32)(includes o144 p37)(includes o144 p38)(includes o144 p39)(includes o144 p72)(includes o144 p124)(includes o144 p134)(includes o144 p169)

(waiting o145)
(includes o145 p10)(includes o145 p93)(includes o145 p120)(includes o145 p121)(includes o145 p154)

(waiting o146)
(includes o146 p23)(includes o146 p81)(includes o146 p86)(includes o146 p103)(includes o146 p128)

(waiting o147)
(includes o147 p29)(includes o147 p41)(includes o147 p66)(includes o147 p110)(includes o147 p122)

(waiting o148)
(includes o148 p25)(includes o148 p51)(includes o148 p107)

(waiting o149)
(includes o149 p62)(includes o149 p68)(includes o149 p144)(includes o149 p162)(includes o149 p173)(includes o149 p178)(includes o149 p190)

(waiting o150)
(includes o150 p1)(includes o150 p55)(includes o150 p115)

(waiting o151)
(includes o151 p84)

(waiting o152)
(includes o152 p42)(includes o152 p113)

(waiting o153)
(includes o153 p1)(includes o153 p23)(includes o153 p65)(includes o153 p104)(includes o153 p163)

(waiting o154)
(includes o154 p28)(includes o154 p153)(includes o154 p180)

(waiting o155)
(includes o155 p12)(includes o155 p25)(includes o155 p45)(includes o155 p90)(includes o155 p122)(includes o155 p127)(includes o155 p138)(includes o155 p164)

(waiting o156)
(includes o156 p7)(includes o156 p21)(includes o156 p84)(includes o156 p111)(includes o156 p122)(includes o156 p156)(includes o156 p184)

(waiting o157)
(includes o157 p34)(includes o157 p47)(includes o157 p57)(includes o157 p96)

(waiting o158)
(includes o158 p25)(includes o158 p44)(includes o158 p75)

(waiting o159)
(includes o159 p7)(includes o159 p35)(includes o159 p41)(includes o159 p54)(includes o159 p68)(includes o159 p155)(includes o159 p175)

(waiting o160)
(includes o160 p30)(includes o160 p47)(includes o160 p96)(includes o160 p102)

(waiting o161)
(includes o161 p8)(includes o161 p12)(includes o161 p31)(includes o161 p115)(includes o161 p158)(includes o161 p161)(includes o161 p168)

(waiting o162)
(includes o162 p39)(includes o162 p47)(includes o162 p157)(includes o162 p182)

(waiting o163)
(includes o163 p59)(includes o163 p85)(includes o163 p167)(includes o163 p181)

(waiting o164)
(includes o164 p31)(includes o164 p64)(includes o164 p101)(includes o164 p119)

(waiting o165)
(includes o165 p94)(includes o165 p185)

(waiting o166)
(includes o166 p106)(includes o166 p174)

(waiting o167)
(includes o167 p16)(includes o167 p48)(includes o167 p142)

(waiting o168)
(includes o168 p49)(includes o168 p117)(includes o168 p123)

(waiting o169)
(includes o169 p6)(includes o169 p9)(includes o169 p91)(includes o169 p105)

(waiting o170)
(includes o170 p19)(includes o170 p40)(includes o170 p45)(includes o170 p51)(includes o170 p68)(includes o170 p85)(includes o170 p91)(includes o170 p149)(includes o170 p165)

(waiting o171)
(includes o171 p33)(includes o171 p71)(includes o171 p79)(includes o171 p90)(includes o171 p99)(includes o171 p123)(includes o171 p152)(includes o171 p184)(includes o171 p187)

(waiting o172)
(includes o172 p9)(includes o172 p106)(includes o172 p138)

(waiting o173)
(includes o173 p8)(includes o173 p43)(includes o173 p55)(includes o173 p102)(includes o173 p145)(includes o173 p155)(includes o173 p181)

(waiting o174)
(includes o174 p48)(includes o174 p104)(includes o174 p137)(includes o174 p164)

(waiting o175)
(includes o175 p19)(includes o175 p78)(includes o175 p113)

(waiting o176)
(includes o176 p4)(includes o176 p36)(includes o176 p43)(includes o176 p90)(includes o176 p160)

(waiting o177)
(includes o177 p13)(includes o177 p34)(includes o177 p44)(includes o177 p45)(includes o177 p129)(includes o177 p135)(includes o177 p159)(includes o177 p181)

(waiting o178)
(includes o178 p1)(includes o178 p37)(includes o178 p54)(includes o178 p64)(includes o178 p69)(includes o178 p98)(includes o178 p137)(includes o178 p150)(includes o178 p185)

(waiting o179)
(includes o179 p29)(includes o179 p58)(includes o179 p62)(includes o179 p74)(includes o179 p94)

(waiting o180)
(includes o180 p1)(includes o180 p23)(includes o180 p26)(includes o180 p36)(includes o180 p42)(includes o180 p53)(includes o180 p94)(includes o180 p97)(includes o180 p111)(includes o180 p139)(includes o180 p165)

(waiting o181)
(includes o181 p28)(includes o181 p29)(includes o181 p31)(includes o181 p81)(includes o181 p104)(includes o181 p142)

(waiting o182)
(includes o182 p81)(includes o182 p138)(includes o182 p154)

(waiting o183)
(includes o183 p5)(includes o183 p36)(includes o183 p38)(includes o183 p60)(includes o183 p92)(includes o183 p99)(includes o183 p121)(includes o183 p149)

(waiting o184)
(includes o184 p48)(includes o184 p117)(includes o184 p124)(includes o184 p126)(includes o184 p132)(includes o184 p184)

(waiting o185)
(includes o185 p8)(includes o185 p32)(includes o185 p68)(includes o185 p72)(includes o185 p116)(includes o185 p171)

(waiting o186)
(includes o186 p17)(includes o186 p102)(includes o186 p151)

(waiting o187)
(includes o187 p2)(includes o187 p17)(includes o187 p126)(includes o187 p131)(includes o187 p152)(includes o187 p157)(includes o187 p171)(includes o187 p174)

(waiting o188)
(includes o188 p5)(includes o188 p14)(includes o188 p29)(includes o188 p66)(includes o188 p85)(includes o188 p134)(includes o188 p179)

(waiting o189)
(includes o189 p31)(includes o189 p32)(includes o189 p58)(includes o189 p105)(includes o189 p114)(includes o189 p135)(includes o189 p138)

(waiting o190)
(includes o190 p110)(includes o190 p111)(includes o190 p144)

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
(not-made p171)
(not-made p172)
(not-made p173)
(not-made p174)
(not-made p175)
(not-made p176)
(not-made p177)
(not-made p178)
(not-made p179)
(not-made p180)
(not-made p181)
(not-made p182)
(not-made p183)
(not-made p184)
(not-made p185)
(not-made p186)
(not-made p187)
(not-made p188)
(not-made p189)
(not-made p190)

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
(shipped o171)
(shipped o172)
(shipped o173)
(shipped o174)
(shipped o175)
(shipped o176)
(shipped o177)
(shipped o178)
(shipped o179)
(shipped o180)
(shipped o181)
(shipped o182)
(shipped o183)
(shipped o184)
(shipped o185)
(shipped o186)
(shipped o187)
(shipped o188)
(shipped o189)
(shipped o190)
))

(:metric minimize (total-cost))

)


(define (problem os-sequencedstrips-p210_1)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) 
(stacks-avail n0)

(waiting o1)
(includes o1 p22)(includes o1 p95)(includes o1 p119)(includes o1 p207)

(waiting o2)
(includes o2 p41)(includes o2 p62)(includes o2 p85)(includes o2 p95)(includes o2 p115)(includes o2 p151)(includes o2 p171)

(waiting o3)
(includes o3 p23)(includes o3 p34)(includes o3 p52)(includes o3 p135)(includes o3 p189)

(waiting o4)
(includes o4 p197)

(waiting o5)
(includes o5 p24)(includes o5 p29)(includes o5 p58)(includes o5 p69)(includes o5 p113)(includes o5 p174)

(waiting o6)
(includes o6 p9)(includes o6 p56)(includes o6 p58)(includes o6 p63)(includes o6 p108)(includes o6 p126)(includes o6 p147)

(waiting o7)
(includes o7 p4)(includes o7 p7)(includes o7 p46)(includes o7 p47)(includes o7 p54)(includes o7 p71)(includes o7 p162)(includes o7 p204)(includes o7 p208)

(waiting o8)
(includes o8 p17)(includes o8 p27)(includes o8 p49)(includes o8 p59)(includes o8 p75)(includes o8 p102)(includes o8 p128)(includes o8 p191)(includes o8 p197)

(waiting o9)
(includes o9 p123)(includes o9 p137)(includes o9 p163)

(waiting o10)
(includes o10 p7)(includes o10 p38)(includes o10 p43)(includes o10 p143)(includes o10 p157)(includes o10 p197)

(waiting o11)
(includes o11 p68)(includes o11 p177)(includes o11 p181)(includes o11 p200)

(waiting o12)
(includes o12 p8)(includes o12 p44)(includes o12 p60)(includes o12 p81)(includes o12 p113)(includes o12 p159)(includes o12 p173)(includes o12 p182)

(waiting o13)
(includes o13 p8)(includes o13 p25)(includes o13 p109)(includes o13 p190)(includes o13 p197)

(waiting o14)
(includes o14 p32)(includes o14 p39)(includes o14 p90)(includes o14 p201)

(waiting o15)
(includes o15 p5)(includes o15 p51)(includes o15 p98)(includes o15 p139)(includes o15 p172)(includes o15 p186)(includes o15 p189)(includes o15 p202)

(waiting o16)
(includes o16 p28)

(waiting o17)
(includes o17 p2)(includes o17 p19)(includes o17 p20)(includes o17 p25)(includes o17 p27)(includes o17 p37)(includes o17 p60)(includes o17 p61)(includes o17 p95)(includes o17 p114)(includes o17 p136)(includes o17 p138)(includes o17 p159)(includes o17 p168)(includes o17 p183)(includes o17 p203)

(waiting o18)
(includes o18 p41)(includes o18 p47)(includes o18 p61)(includes o18 p76)

(waiting o19)
(includes o19 p43)(includes o19 p103)(includes o19 p128)(includes o19 p136)(includes o19 p170)(includes o19 p179)(includes o19 p207)

(waiting o20)
(includes o20 p17)(includes o20 p25)(includes o20 p50)(includes o20 p91)(includes o20 p109)(includes o20 p117)(includes o20 p146)

(waiting o21)
(includes o21 p12)(includes o21 p205)

(waiting o22)
(includes o22 p3)(includes o22 p7)(includes o22 p39)(includes o22 p44)(includes o22 p73)(includes o22 p79)(includes o22 p86)(includes o22 p100)(includes o22 p112)(includes o22 p193)

(waiting o23)
(includes o23 p48)(includes o23 p60)(includes o23 p62)(includes o23 p73)(includes o23 p74)(includes o23 p94)(includes o23 p145)(includes o23 p178)

(waiting o24)
(includes o24 p14)(includes o24 p87)(includes o24 p99)(includes o24 p117)(includes o24 p196)(includes o24 p199)

(waiting o25)
(includes o25 p31)(includes o25 p61)(includes o25 p83)(includes o25 p169)

(waiting o26)
(includes o26 p28)(includes o26 p127)(includes o26 p142)(includes o26 p153)

(waiting o27)
(includes o27 p53)(includes o27 p106)(includes o27 p164)(includes o27 p174)(includes o27 p189)

(waiting o28)
(includes o28 p79)(includes o28 p87)(includes o28 p93)(includes o28 p140)(includes o28 p184)

(waiting o29)
(includes o29 p11)(includes o29 p26)(includes o29 p76)(includes o29 p91)(includes o29 p95)(includes o29 p113)(includes o29 p140)(includes o29 p149)(includes o29 p199)

(waiting o30)
(includes o30 p8)(includes o30 p10)(includes o30 p15)(includes o30 p101)(includes o30 p187)(includes o30 p188)

(waiting o31)
(includes o31 p13)(includes o31 p20)(includes o31 p37)(includes o31 p82)(includes o31 p110)(includes o31 p133)(includes o31 p178)

(waiting o32)
(includes o32 p22)(includes o32 p35)(includes o32 p51)(includes o32 p93)(includes o32 p109)(includes o32 p115)(includes o32 p128)(includes o32 p145)(includes o32 p177)

(waiting o33)
(includes o33 p86)(includes o33 p88)(includes o33 p178)(includes o33 p185)

(waiting o34)
(includes o34 p4)(includes o34 p50)(includes o34 p109)(includes o34 p146)(includes o34 p150)(includes o34 p151)(includes o34 p201)

(waiting o35)
(includes o35 p36)(includes o35 p62)(includes o35 p110)(includes o35 p200)

(waiting o36)
(includes o36 p24)(includes o36 p175)

(waiting o37)
(includes o37 p126)(includes o37 p151)

(waiting o38)
(includes o38 p15)(includes o38 p40)(includes o38 p132)(includes o38 p189)

(waiting o39)
(includes o39 p5)(includes o39 p12)(includes o39 p14)(includes o39 p74)(includes o39 p92)(includes o39 p168)(includes o39 p191)

(waiting o40)
(includes o40 p9)(includes o40 p30)(includes o40 p40)(includes o40 p123)(includes o40 p145)(includes o40 p180)(includes o40 p201)(includes o40 p203)

(waiting o41)
(includes o41 p23)(includes o41 p147)(includes o41 p166)(includes o41 p183)(includes o41 p184)

(waiting o42)
(includes o42 p69)(includes o42 p91)(includes o42 p137)(includes o42 p148)(includes o42 p189)(includes o42 p208)

(waiting o43)
(includes o43 p23)(includes o43 p41)(includes o43 p56)(includes o43 p57)(includes o43 p61)(includes o43 p87)(includes o43 p169)(includes o43 p181)

(waiting o44)
(includes o44 p10)(includes o44 p22)(includes o44 p38)(includes o44 p88)(includes o44 p89)(includes o44 p103)(includes o44 p123)(includes o44 p156)(includes o44 p178)(includes o44 p181)(includes o44 p190)

(waiting o45)
(includes o45 p10)(includes o45 p16)(includes o45 p24)(includes o45 p53)(includes o45 p111)(includes o45 p165)(includes o45 p170)

(waiting o46)
(includes o46 p24)(includes o46 p57)(includes o46 p71)(includes o46 p123)(includes o46 p177)(includes o46 p190)(includes o46 p205)

(waiting o47)
(includes o47 p1)(includes o47 p205)

(waiting o48)
(includes o48 p9)(includes o48 p37)(includes o48 p56)(includes o48 p166)(includes o48 p188)(includes o48 p207)

(waiting o49)
(includes o49 p12)(includes o49 p50)(includes o49 p61)(includes o49 p63)(includes o49 p78)(includes o49 p101)(includes o49 p111)(includes o49 p141)(includes o49 p146)(includes o49 p178)(includes o49 p184)(includes o49 p192)(includes o49 p204)

(waiting o50)
(includes o50 p12)(includes o50 p91)(includes o50 p97)(includes o50 p127)(includes o50 p156)(includes o50 p160)(includes o50 p162)(includes o50 p192)

(waiting o51)
(includes o51 p60)(includes o51 p80)(includes o51 p92)(includes o51 p118)

(waiting o52)
(includes o52 p20)(includes o52 p72)(includes o52 p86)(includes o52 p93)(includes o52 p126)

(waiting o53)
(includes o53 p4)(includes o53 p14)(includes o53 p34)(includes o53 p64)(includes o53 p77)(includes o53 p88)(includes o53 p107)

(waiting o54)
(includes o54 p7)(includes o54 p11)(includes o54 p66)(includes o54 p69)(includes o54 p95)(includes o54 p120)(includes o54 p206)

(waiting o55)
(includes o55 p95)(includes o55 p104)(includes o55 p115)(includes o55 p146)(includes o55 p152)(includes o55 p158)(includes o55 p202)

(waiting o56)
(includes o56 p11)(includes o56 p15)(includes o56 p52)(includes o56 p82)(includes o56 p95)(includes o56 p115)(includes o56 p120)(includes o56 p146)(includes o56 p152)(includes o56 p209)

(waiting o57)
(includes o57 p15)(includes o57 p44)(includes o57 p91)(includes o57 p102)(includes o57 p106)(includes o57 p160)(includes o57 p173)(includes o57 p203)(includes o57 p209)

(waiting o58)
(includes o58 p14)(includes o58 p95)(includes o58 p130)

(waiting o59)
(includes o59 p22)(includes o59 p31)(includes o59 p33)(includes o59 p65)(includes o59 p72)(includes o59 p78)(includes o59 p132)(includes o59 p145)(includes o59 p154)(includes o59 p197)

(waiting o60)
(includes o60 p53)(includes o60 p111)(includes o60 p116)(includes o60 p142)(includes o60 p199)

(waiting o61)
(includes o61 p11)(includes o61 p129)(includes o61 p149)

(waiting o62)
(includes o62 p13)(includes o62 p39)(includes o62 p142)(includes o62 p199)(includes o62 p208)

(waiting o63)
(includes o63 p52)(includes o63 p53)(includes o63 p79)(includes o63 p110)(includes o63 p132)(includes o63 p189)

(waiting o64)
(includes o64 p74)(includes o64 p114)(includes o64 p142)

(waiting o65)
(includes o65 p9)(includes o65 p10)(includes o65 p31)(includes o65 p36)(includes o65 p44)(includes o65 p151)(includes o65 p190)(includes o65 p203)

(waiting o66)
(includes o66 p70)(includes o66 p109)(includes o66 p116)(includes o66 p127)(includes o66 p131)(includes o66 p141)(includes o66 p147)(includes o66 p204)

(waiting o67)
(includes o67 p33)(includes o67 p190)

(waiting o68)
(includes o68 p59)(includes o68 p65)(includes o68 p104)(includes o68 p123)(includes o68 p131)(includes o68 p141)(includes o68 p154)(includes o68 p204)

(waiting o69)
(includes o69 p31)(includes o69 p47)(includes o69 p53)(includes o69 p69)(includes o69 p147)(includes o69 p177)(includes o69 p179)

(waiting o70)
(includes o70 p47)(includes o70 p98)

(waiting o71)
(includes o71 p114)(includes o71 p145)(includes o71 p180)

(waiting o72)
(includes o72 p25)(includes o72 p96)(includes o72 p100)(includes o72 p117)

(waiting o73)
(includes o73 p9)(includes o73 p77)(includes o73 p128)(includes o73 p151)

(waiting o74)
(includes o74 p52)(includes o74 p56)(includes o74 p173)(includes o74 p190)

(waiting o75)
(includes o75 p50)(includes o75 p127)

(waiting o76)
(includes o76 p4)(includes o76 p67)(includes o76 p104)(includes o76 p105)

(waiting o77)
(includes o77 p8)(includes o77 p101)(includes o77 p142)(includes o77 p146)

(waiting o78)
(includes o78 p8)(includes o78 p129)(includes o78 p148)(includes o78 p156)(includes o78 p200)

(waiting o79)
(includes o79 p84)(includes o79 p90)(includes o79 p92)(includes o79 p110)(includes o79 p127)(includes o79 p181)

(waiting o80)
(includes o80 p7)(includes o80 p26)(includes o80 p35)(includes o80 p81)(includes o80 p111)(includes o80 p200)

(waiting o81)
(includes o81 p16)(includes o81 p91)(includes o81 p93)(includes o81 p98)(includes o81 p130)(includes o81 p178)

(waiting o82)
(includes o82 p16)(includes o82 p69)(includes o82 p103)(includes o82 p117)(includes o82 p143)(includes o82 p161)(includes o82 p167)(includes o82 p205)

(waiting o83)
(includes o83 p49)(includes o83 p66)(includes o83 p72)(includes o83 p91)(includes o83 p113)(includes o83 p175)(includes o83 p202)(includes o83 p206)(includes o83 p209)

(waiting o84)
(includes o84 p16)(includes o84 p17)(includes o84 p58)(includes o84 p63)(includes o84 p108)(includes o84 p156)

(waiting o85)
(includes o85 p90)(includes o85 p193)

(waiting o86)
(includes o86 p17)(includes o86 p18)(includes o86 p21)(includes o86 p54)(includes o86 p85)(includes o86 p93)(includes o86 p165)

(waiting o87)
(includes o87 p37)(includes o87 p56)(includes o87 p99)(includes o87 p162)(includes o87 p186)

(waiting o88)
(includes o88 p88)(includes o88 p126)(includes o88 p136)(includes o88 p146)

(waiting o89)
(includes o89 p15)(includes o89 p42)(includes o89 p50)(includes o89 p112)

(waiting o90)
(includes o90 p68)(includes o90 p74)(includes o90 p82)(includes o90 p96)(includes o90 p159)(includes o90 p174)(includes o90 p192)(includes o90 p193)(includes o90 p197)

(waiting o91)
(includes o91 p21)(includes o91 p99)(includes o91 p135)(includes o91 p198)

(waiting o92)
(includes o92 p58)(includes o92 p64)(includes o92 p74)(includes o92 p82)(includes o92 p86)(includes o92 p87)(includes o92 p97)(includes o92 p111)(includes o92 p170)(includes o92 p202)

(waiting o93)
(includes o93 p7)(includes o93 p15)(includes o93 p66)(includes o93 p76)(includes o93 p92)(includes o93 p138)(includes o93 p144)(includes o93 p168)(includes o93 p207)(includes o93 p208)

(waiting o94)
(includes o94 p3)(includes o94 p6)(includes o94 p23)(includes o94 p79)(includes o94 p93)(includes o94 p106)(includes o94 p142)

(waiting o95)
(includes o95 p15)(includes o95 p25)(includes o95 p32)(includes o95 p138)(includes o95 p162)(includes o95 p168)(includes o95 p202)

(waiting o96)
(includes o96 p16)(includes o96 p70)(includes o96 p125)(includes o96 p167)

(waiting o97)
(includes o97 p26)(includes o97 p30)(includes o97 p129)(includes o97 p182)

(waiting o98)
(includes o98 p12)

(waiting o99)
(includes o99 p4)(includes o99 p18)(includes o99 p20)

(waiting o100)
(includes o100 p22)(includes o100 p30)(includes o100 p57)(includes o100 p72)(includes o100 p83)(includes o100 p109)(includes o100 p147)(includes o100 p157)(includes o100 p187)

(waiting o101)
(includes o101 p92)(includes o101 p99)(includes o101 p147)(includes o101 p199)(includes o101 p204)

(waiting o102)
(includes o102 p20)(includes o102 p85)(includes o102 p114)(includes o102 p124)(includes o102 p146)(includes o102 p157)(includes o102 p201)

(waiting o103)
(includes o103 p21)(includes o103 p33)(includes o103 p47)(includes o103 p93)(includes o103 p118)(includes o103 p123)(includes o103 p179)

(waiting o104)
(includes o104 p10)(includes o104 p94)(includes o104 p101)(includes o104 p150)(includes o104 p158)

(waiting o105)
(includes o105 p37)(includes o105 p55)(includes o105 p60)(includes o105 p111)(includes o105 p124)

(waiting o106)
(includes o106 p17)(includes o106 p77)(includes o106 p103)(includes o106 p118)(includes o106 p122)(includes o106 p123)(includes o106 p201)

(waiting o107)
(includes o107 p100)(includes o107 p207)

(waiting o108)
(includes o108 p56)(includes o108 p77)(includes o108 p131)(includes o108 p171)

(waiting o109)
(includes o109 p23)(includes o109 p51)(includes o109 p82)(includes o109 p100)(includes o109 p131)(includes o109 p165)

(waiting o110)
(includes o110 p20)(includes o110 p42)(includes o110 p58)(includes o110 p108)(includes o110 p159)(includes o110 p160)

(waiting o111)
(includes o111 p5)(includes o111 p81)(includes o111 p101)(includes o111 p170)(includes o111 p172)

(waiting o112)
(includes o112 p19)(includes o112 p47)(includes o112 p48)(includes o112 p65)(includes o112 p154)(includes o112 p164)(includes o112 p174)

(waiting o113)
(includes o113 p3)(includes o113 p31)(includes o113 p42)(includes o113 p72)(includes o113 p83)(includes o113 p148)(includes o113 p158)(includes o113 p168)(includes o113 p172)(includes o113 p174)(includes o113 p190)(includes o113 p195)(includes o113 p199)

(waiting o114)
(includes o114 p160)(includes o114 p162)(includes o114 p191)

(waiting o115)
(includes o115 p67)(includes o115 p69)(includes o115 p72)(includes o115 p126)(includes o115 p135)(includes o115 p142)(includes o115 p198)

(waiting o116)
(includes o116 p18)(includes o116 p47)(includes o116 p93)(includes o116 p109)(includes o116 p118)(includes o116 p136)(includes o116 p140)(includes o116 p154)(includes o116 p171)(includes o116 p185)(includes o116 p204)

(waiting o117)
(includes o117 p9)(includes o117 p72)(includes o117 p98)(includes o117 p121)(includes o117 p123)

(waiting o118)
(includes o118 p74)(includes o118 p96)(includes o118 p109)(includes o118 p140)(includes o118 p175)(includes o118 p180)(includes o118 p201)

(waiting o119)
(includes o119 p23)(includes o119 p91)(includes o119 p92)(includes o119 p140)

(waiting o120)
(includes o120 p11)(includes o120 p76)(includes o120 p107)(includes o120 p132)(includes o120 p162)(includes o120 p206)

(waiting o121)
(includes o121 p4)(includes o121 p36)(includes o121 p38)(includes o121 p45)(includes o121 p61)(includes o121 p69)(includes o121 p88)(includes o121 p134)(includes o121 p153)(includes o121 p157)(includes o121 p171)(includes o121 p201)

(waiting o122)
(includes o122 p10)(includes o122 p25)(includes o122 p31)(includes o122 p95)(includes o122 p103)(includes o122 p107)(includes o122 p129)(includes o122 p140)(includes o122 p195)(includes o122 p207)

(waiting o123)
(includes o123 p53)(includes o123 p66)(includes o123 p90)(includes o123 p113)(includes o123 p151)(includes o123 p168)

(waiting o124)
(includes o124 p20)(includes o124 p72)(includes o124 p85)(includes o124 p109)(includes o124 p170)(includes o124 p177)(includes o124 p186)(includes o124 p203)

(waiting o125)
(includes o125 p40)(includes o125 p73)(includes o125 p84)(includes o125 p85)(includes o125 p89)(includes o125 p152)(includes o125 p160)(includes o125 p179)(includes o125 p185)

(waiting o126)
(includes o126 p64)(includes o126 p65)(includes o126 p164)(includes o126 p178)

(waiting o127)
(includes o127 p48)(includes o127 p78)(includes o127 p148)(includes o127 p174)

(waiting o128)
(includes o128 p24)(includes o128 p72)(includes o128 p79)(includes o128 p111)(includes o128 p177)(includes o128 p208)

(waiting o129)
(includes o129 p57)(includes o129 p96)(includes o129 p197)

(waiting o130)
(includes o130 p6)(includes o130 p18)(includes o130 p22)(includes o130 p62)(includes o130 p63)(includes o130 p89)

(waiting o131)
(includes o131 p6)(includes o131 p97)(includes o131 p157)(includes o131 p197)

(waiting o132)
(includes o132 p64)(includes o132 p124)(includes o132 p162)(includes o132 p195)

(waiting o133)
(includes o133 p57)(includes o133 p84)(includes o133 p101)(includes o133 p113)(includes o133 p161)(includes o133 p164)(includes o133 p186)(includes o133 p188)

(waiting o134)
(includes o134 p23)(includes o134 p36)(includes o134 p40)(includes o134 p113)(includes o134 p125)(includes o134 p191)

(waiting o135)
(includes o135 p67)(includes o135 p119)(includes o135 p134)

(waiting o136)
(includes o136 p122)(includes o136 p129)(includes o136 p148)(includes o136 p156)(includes o136 p196)(includes o136 p209)

(waiting o137)
(includes o137 p1)(includes o137 p18)(includes o137 p21)(includes o137 p33)(includes o137 p40)(includes o137 p46)(includes o137 p74)(includes o137 p75)(includes o137 p176)(includes o137 p187)

(waiting o138)
(includes o138 p45)

(waiting o139)
(includes o139 p20)(includes o139 p22)(includes o139 p57)(includes o139 p93)(includes o139 p110)(includes o139 p167)(includes o139 p169)

(waiting o140)
(includes o140 p99)(includes o140 p142)(includes o140 p160)(includes o140 p165)(includes o140 p176)(includes o140 p191)

(waiting o141)
(includes o141 p38)(includes o141 p133)(includes o141 p160)(includes o141 p168)(includes o141 p208)

(waiting o142)
(includes o142 p30)(includes o142 p72)(includes o142 p144)(includes o142 p150)(includes o142 p206)

(waiting o143)
(includes o143 p84)(includes o143 p109)

(waiting o144)
(includes o144 p9)(includes o144 p170)(includes o144 p171)(includes o144 p192)

(waiting o145)
(includes o145 p13)(includes o145 p51)(includes o145 p68)(includes o145 p83)(includes o145 p174)

(waiting o146)
(includes o146 p42)(includes o146 p47)(includes o146 p118)

(waiting o147)
(includes o147 p77)(includes o147 p120)(includes o147 p128)(includes o147 p158)(includes o147 p181)(includes o147 p194)(includes o147 p195)(includes o147 p198)(includes o147 p202)(includes o147 p208)

(waiting o148)
(includes o148 p26)(includes o148 p31)(includes o148 p144)(includes o148 p182)(includes o148 p206)

(waiting o149)
(includes o149 p3)(includes o149 p38)(includes o149 p88)(includes o149 p100)(includes o149 p124)

(waiting o150)
(includes o150 p13)(includes o150 p72)(includes o150 p164)(includes o150 p185)(includes o150 p201)

(waiting o151)
(includes o151 p10)(includes o151 p11)(includes o151 p23)(includes o151 p134)(includes o151 p164)(includes o151 p194)(includes o151 p205)

(waiting o152)
(includes o152 p10)(includes o152 p26)(includes o152 p56)(includes o152 p63)(includes o152 p79)(includes o152 p114)(includes o152 p121)(includes o152 p166)(includes o152 p173)

(waiting o153)
(includes o153 p20)(includes o153 p87)(includes o153 p91)

(waiting o154)
(includes o154 p63)(includes o154 p195)

(waiting o155)
(includes o155 p17)(includes o155 p21)(includes o155 p61)(includes o155 p119)(includes o155 p142)(includes o155 p158)(includes o155 p186)(includes o155 p202)

(waiting o156)
(includes o156 p34)(includes o156 p55)(includes o156 p61)(includes o156 p68)(includes o156 p91)(includes o156 p105)(includes o156 p178)(includes o156 p180)(includes o156 p184)(includes o156 p190)(includes o156 p210)

(waiting o157)
(includes o157 p5)(includes o157 p39)(includes o157 p100)(includes o157 p164)

(waiting o158)
(includes o158 p2)(includes o158 p49)(includes o158 p98)(includes o158 p129)(includes o158 p187)

(waiting o159)
(includes o159 p59)(includes o159 p75)(includes o159 p141)(includes o159 p189)(includes o159 p204)

(waiting o160)
(includes o160 p150)

(waiting o161)
(includes o161 p16)(includes o161 p57)(includes o161 p70)(includes o161 p81)(includes o161 p90)(includes o161 p124)(includes o161 p128)(includes o161 p173)(includes o161 p174)(includes o161 p197)

(waiting o162)
(includes o162 p3)(includes o162 p17)(includes o162 p44)(includes o162 p58)(includes o162 p88)(includes o162 p114)(includes o162 p137)(includes o162 p164)(includes o162 p180)

(waiting o163)
(includes o163 p15)(includes o163 p71)(includes o163 p103)(includes o163 p115)(includes o163 p195)(includes o163 p208)

(waiting o164)
(includes o164 p100)(includes o164 p158)(includes o164 p168)(includes o164 p200)

(waiting o165)
(includes o165 p32)(includes o165 p71)(includes o165 p79)(includes o165 p90)(includes o165 p98)(includes o165 p142)

(waiting o166)
(includes o166 p41)(includes o166 p50)(includes o166 p66)(includes o166 p140)(includes o166 p143)(includes o166 p155)(includes o166 p161)(includes o166 p206)

(waiting o167)
(includes o167 p89)(includes o167 p112)

(waiting o168)
(includes o168 p2)(includes o168 p17)(includes o168 p27)(includes o168 p50)(includes o168 p92)(includes o168 p113)(includes o168 p155)

(waiting o169)
(includes o169 p10)(includes o169 p36)(includes o169 p71)(includes o169 p74)(includes o169 p86)(includes o169 p94)

(waiting o170)
(includes o170 p31)(includes o170 p77)(includes o170 p98)(includes o170 p153)(includes o170 p166)(includes o170 p197)

(waiting o171)
(includes o171 p4)(includes o171 p49)(includes o171 p108)(includes o171 p117)(includes o171 p121)(includes o171 p151)(includes o171 p201)(includes o171 p204)(includes o171 p208)

(waiting o172)
(includes o172 p26)(includes o172 p92)(includes o172 p151)(includes o172 p183)(includes o172 p191)

(waiting o173)
(includes o173 p140)(includes o173 p157)(includes o173 p207)

(waiting o174)
(includes o174 p38)(includes o174 p41)(includes o174 p64)(includes o174 p81)(includes o174 p90)(includes o174 p119)(includes o174 p206)

(waiting o175)
(includes o175 p54)(includes o175 p183)

(waiting o176)
(includes o176 p58)(includes o176 p96)(includes o176 p106)(includes o176 p200)

(waiting o177)
(includes o177 p37)(includes o177 p109)(includes o177 p114)(includes o177 p188)

(waiting o178)
(includes o178 p35)(includes o178 p58)(includes o178 p59)(includes o178 p101)(includes o178 p135)(includes o178 p138)(includes o178 p165)

(waiting o179)
(includes o179 p9)(includes o179 p50)(includes o179 p76)(includes o179 p123)(includes o179 p202)

(waiting o180)
(includes o180 p92)(includes o180 p148)(includes o180 p194)

(waiting o181)
(includes o181 p26)(includes o181 p60)(includes o181 p83)(includes o181 p189)(includes o181 p193)(includes o181 p202)

(waiting o182)
(includes o182 p8)(includes o182 p37)(includes o182 p45)(includes o182 p50)(includes o182 p82)(includes o182 p111)

(waiting o183)
(includes o183 p67)(includes o183 p124)(includes o183 p176)

(waiting o184)
(includes o184 p68)(includes o184 p101)(includes o184 p167)(includes o184 p172)(includes o184 p203)

(waiting o185)
(includes o185 p40)(includes o185 p81)(includes o185 p131)(includes o185 p155)

(waiting o186)
(includes o186 p29)(includes o186 p42)(includes o186 p73)(includes o186 p133)

(waiting o187)
(includes o187 p19)(includes o187 p21)(includes o187 p52)(includes o187 p133)(includes o187 p134)(includes o187 p138)(includes o187 p165)

(waiting o188)
(includes o188 p7)(includes o188 p82)(includes o188 p96)

(waiting o189)
(includes o189 p8)(includes o189 p55)(includes o189 p68)(includes o189 p76)(includes o189 p89)(includes o189 p106)(includes o189 p139)(includes o189 p208)

(waiting o190)
(includes o190 p47)(includes o190 p48)(includes o190 p68)(includes o190 p73)(includes o190 p101)(includes o190 p109)(includes o190 p157)(includes o190 p179)(includes o190 p188)

(waiting o191)
(includes o191 p52)(includes o191 p58)(includes o191 p64)(includes o191 p134)(includes o191 p144)(includes o191 p151)

(waiting o192)
(includes o192 p35)(includes o192 p70)

(waiting o193)
(includes o193 p15)(includes o193 p23)(includes o193 p25)(includes o193 p65)(includes o193 p68)(includes o193 p134)(includes o193 p138)

(waiting o194)
(includes o194 p120)(includes o194 p162)(includes o194 p168)(includes o194 p189)

(waiting o195)
(includes o195 p53)(includes o195 p81)(includes o195 p118)(includes o195 p175)

(waiting o196)
(includes o196 p67)(includes o196 p80)(includes o196 p116)(includes o196 p119)(includes o196 p122)(includes o196 p142)(includes o196 p165)

(waiting o197)
(includes o197 p19)(includes o197 p48)(includes o197 p57)(includes o197 p74)(includes o197 p82)(includes o197 p93)(includes o197 p123)(includes o197 p152)(includes o197 p171)(includes o197 p181)(includes o197 p185)

(waiting o198)
(includes o198 p4)(includes o198 p142)(includes o198 p180)

(waiting o199)
(includes o199 p99)(includes o199 p107)(includes o199 p151)(includes o199 p200)

(waiting o200)
(includes o200 p11)(includes o200 p16)(includes o200 p25)(includes o200 p29)(includes o200 p95)(includes o200 p139)(includes o200 p190)

(waiting o201)
(includes o201 p37)(includes o201 p38)(includes o201 p88)(includes o201 p106)(includes o201 p119)(includes o201 p121)(includes o201 p167)

(waiting o202)
(includes o202 p15)(includes o202 p18)(includes o202 p19)(includes o202 p35)(includes o202 p67)(includes o202 p81)(includes o202 p123)(includes o202 p161)(includes o202 p171)

(waiting o203)
(includes o203 p58)(includes o203 p107)(includes o203 p120)(includes o203 p122)(includes o203 p129)(includes o203 p130)(includes o203 p151)(includes o203 p181)(includes o203 p193)(includes o203 p195)(includes o203 p205)

(waiting o204)
(includes o204 p47)(includes o204 p114)(includes o204 p115)(includes o204 p144)(includes o204 p153)(includes o204 p207)

(waiting o205)
(includes o205 p78)

(waiting o206)
(includes o206 p4)(includes o206 p9)(includes o206 p27)(includes o206 p72)(includes o206 p194)(includes o206 p208)

(waiting o207)
(includes o207 p16)(includes o207 p22)(includes o207 p29)(includes o207 p78)(includes o207 p134)(includes o207 p179)

(waiting o208)
(includes o208 p50)(includes o208 p62)(includes o208 p77)(includes o208 p83)(includes o208 p110)(includes o208 p127)

(waiting o209)
(includes o209 p45)(includes o209 p51)(includes o209 p113)(includes o209 p118)(includes o209 p133)

(waiting o210)
(includes o210 p19)(includes o210 p37)(includes o210 p94)(includes o210 p146)

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
(not-made p191)
(not-made p192)
(not-made p193)
(not-made p194)
(not-made p195)
(not-made p196)
(not-made p197)
(not-made p198)
(not-made p199)
(not-made p200)
(not-made p201)
(not-made p202)
(not-made p203)
(not-made p204)
(not-made p205)
(not-made p206)
(not-made p207)
(not-made p208)
(not-made p209)
(not-made p210)

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
(shipped o191)
(shipped o192)
(shipped o193)
(shipped o194)
(shipped o195)
(shipped o196)
(shipped o197)
(shipped o198)
(shipped o199)
(shipped o200)
(shipped o201)
(shipped o202)
(shipped o203)
(shipped o204)
(shipped o205)
(shipped o206)
(shipped o207)
(shipped o208)
(shipped o209)
(shipped o210)
))

(:metric minimize (total-cost))

)


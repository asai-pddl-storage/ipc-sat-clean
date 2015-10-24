(define (problem os-sequencedstrips-p250_3)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230 n231 n232 n233 n234 n235 n236 n237 n238 n239 n240 n241 n242 n243 n244 n245 n246 n247 n248 n249 n250  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) (next-count n230 n231) (next-count n231 n232) (next-count n232 n233) (next-count n233 n234) (next-count n234 n235) (next-count n235 n236) (next-count n236 n237) (next-count n237 n238) (next-count n238 n239) (next-count n239 n240) (next-count n240 n241) (next-count n241 n242) (next-count n242 n243) (next-count n243 n244) (next-count n244 n245) (next-count n245 n246) (next-count n246 n247) (next-count n247 n248) (next-count n248 n249) (next-count n249 n250) 
(stacks-avail n0)

(waiting o1)
(includes o1 p40)(includes o1 p75)(includes o1 p101)(includes o1 p111)(includes o1 p117)(includes o1 p175)(includes o1 p197)(includes o1 p228)

(waiting o2)
(includes o2 p33)(includes o2 p90)(includes o2 p184)

(waiting o3)
(includes o3 p83)(includes o3 p113)(includes o3 p126)

(waiting o4)
(includes o4 p29)(includes o4 p40)(includes o4 p71)(includes o4 p96)(includes o4 p116)(includes o4 p117)(includes o4 p118)(includes o4 p127)(includes o4 p148)(includes o4 p172)(includes o4 p185)(includes o4 p193)(includes o4 p222)

(waiting o5)
(includes o5 p26)(includes o5 p41)(includes o5 p47)(includes o5 p126)(includes o5 p146)

(waiting o6)
(includes o6 p28)(includes o6 p106)(includes o6 p153)(includes o6 p207)

(waiting o7)
(includes o7 p46)(includes o7 p65)(includes o7 p96)(includes o7 p102)(includes o7 p109)(includes o7 p116)(includes o7 p125)(includes o7 p142)(includes o7 p188)(includes o7 p213)(includes o7 p224)

(waiting o8)
(includes o8 p12)(includes o8 p36)(includes o8 p110)(includes o8 p163)(includes o8 p172)(includes o8 p195)(includes o8 p208)

(waiting o9)
(includes o9 p6)(includes o9 p59)(includes o9 p151)(includes o9 p154)(includes o9 p165)(includes o9 p189)(includes o9 p209)

(waiting o10)
(includes o10 p155)(includes o10 p200)(includes o10 p205)(includes o10 p222)

(waiting o11)
(includes o11 p57)(includes o11 p73)(includes o11 p112)(includes o11 p244)

(waiting o12)
(includes o12 p9)(includes o12 p22)(includes o12 p30)(includes o12 p35)(includes o12 p61)(includes o12 p70)(includes o12 p174)(includes o12 p179)

(waiting o13)
(includes o13 p4)(includes o13 p44)(includes o13 p46)(includes o13 p48)(includes o13 p74)(includes o13 p185)(includes o13 p189)

(waiting o14)
(includes o14 p49)(includes o14 p54)(includes o14 p56)(includes o14 p66)(includes o14 p111)(includes o14 p122)(includes o14 p197)

(waiting o15)
(includes o15 p11)(includes o15 p25)(includes o15 p128)(includes o15 p238)

(waiting o16)
(includes o16 p36)(includes o16 p44)(includes o16 p91)(includes o16 p114)(includes o16 p185)(includes o16 p191)

(waiting o17)
(includes o17 p83)

(waiting o18)
(includes o18 p13)(includes o18 p22)(includes o18 p122)(includes o18 p206)(includes o18 p223)

(waiting o19)
(includes o19 p13)(includes o19 p34)(includes o19 p72)(includes o19 p104)(includes o19 p119)

(waiting o20)
(includes o20 p3)(includes o20 p18)(includes o20 p51)(includes o20 p58)(includes o20 p63)(includes o20 p125)(includes o20 p150)(includes o20 p162)

(waiting o21)
(includes o21 p46)(includes o21 p63)(includes o21 p117)(includes o21 p157)(includes o21 p224)(includes o21 p237)

(waiting o22)
(includes o22 p19)(includes o22 p58)(includes o22 p64)(includes o22 p119)(includes o22 p159)(includes o22 p164)(includes o22 p169)

(waiting o23)
(includes o23 p38)(includes o23 p47)(includes o23 p75)(includes o23 p81)(includes o23 p126)(includes o23 p214)

(waiting o24)
(includes o24 p59)(includes o24 p71)(includes o24 p84)(includes o24 p92)(includes o24 p119)

(waiting o25)
(includes o25 p15)(includes o25 p56)(includes o25 p99)(includes o25 p119)(includes o25 p134)(includes o25 p138)(includes o25 p203)

(waiting o26)
(includes o26 p28)(includes o26 p70)(includes o26 p78)(includes o26 p141)(includes o26 p173)(includes o26 p241)

(waiting o27)
(includes o27 p22)(includes o27 p45)(includes o27 p98)(includes o27 p157)(includes o27 p177)(includes o27 p205)(includes o27 p222)

(waiting o28)
(includes o28 p43)(includes o28 p244)

(waiting o29)
(includes o29 p16)(includes o29 p82)(includes o29 p91)(includes o29 p218)

(waiting o30)
(includes o30 p67)(includes o30 p139)(includes o30 p142)(includes o30 p154)

(waiting o31)
(includes o31 p38)(includes o31 p48)(includes o31 p146)(includes o31 p152)(includes o31 p192)(includes o31 p227)(includes o31 p248)

(waiting o32)
(includes o32 p25)(includes o32 p65)(includes o32 p138)(includes o32 p154)(includes o32 p167)(includes o32 p188)

(waiting o33)
(includes o33 p106)(includes o33 p196)(includes o33 p209)

(waiting o34)
(includes o34 p57)(includes o34 p63)(includes o34 p135)(includes o34 p179)

(waiting o35)
(includes o35 p49)(includes o35 p57)(includes o35 p81)(includes o35 p88)(includes o35 p111)(includes o35 p119)(includes o35 p238)

(waiting o36)
(includes o36 p103)(includes o36 p105)(includes o36 p132)

(waiting o37)
(includes o37 p73)(includes o37 p140)(includes o37 p238)

(waiting o38)
(includes o38 p2)(includes o38 p11)(includes o38 p16)(includes o38 p55)(includes o38 p96)(includes o38 p141)

(waiting o39)
(includes o39 p31)(includes o39 p94)(includes o39 p107)(includes o39 p125)(includes o39 p150)(includes o39 p158)(includes o39 p199)(includes o39 p215)

(waiting o40)
(includes o40 p3)(includes o40 p108)(includes o40 p126)(includes o40 p231)

(waiting o41)
(includes o41 p11)(includes o41 p18)(includes o41 p67)(includes o41 p119)(includes o41 p211)

(waiting o42)
(includes o42 p46)(includes o42 p56)(includes o42 p60)(includes o42 p62)(includes o42 p83)(includes o42 p131)(includes o42 p138)(includes o42 p144)(includes o42 p224)

(waiting o43)
(includes o43 p89)(includes o43 p144)(includes o43 p155)(includes o43 p175)

(waiting o44)
(includes o44 p88)(includes o44 p90)(includes o44 p128)(includes o44 p137)(includes o44 p187)(includes o44 p207)(includes o44 p211)(includes o44 p225)(includes o44 p238)

(waiting o45)
(includes o45 p1)(includes o45 p164)

(waiting o46)
(includes o46 p74)(includes o46 p181)(includes o46 p222)(includes o46 p245)

(waiting o47)
(includes o47 p17)(includes o47 p43)(includes o47 p70)(includes o47 p196)

(waiting o48)
(includes o48 p12)(includes o48 p35)(includes o48 p77)(includes o48 p79)(includes o48 p123)(includes o48 p162)(includes o48 p174)(includes o48 p182)(includes o48 p190)(includes o48 p213)(includes o48 p214)(includes o48 p244)

(waiting o49)
(includes o49 p91)(includes o49 p121)(includes o49 p185)(includes o49 p208)(includes o49 p214)

(waiting o50)
(includes o50 p178)(includes o50 p181)(includes o50 p226)

(waiting o51)
(includes o51 p5)(includes o51 p8)(includes o51 p10)(includes o51 p48)(includes o51 p81)(includes o51 p157)(includes o51 p176)(includes o51 p183)

(waiting o52)
(includes o52 p17)(includes o52 p40)(includes o52 p42)(includes o52 p59)(includes o52 p131)(includes o52 p168)(includes o52 p197)(includes o52 p218)(includes o52 p229)

(waiting o53)
(includes o53 p36)(includes o53 p42)(includes o53 p78)(includes o53 p82)(includes o53 p94)(includes o53 p100)(includes o53 p124)(includes o53 p145)(includes o53 p217)(includes o53 p222)

(waiting o54)
(includes o54 p7)(includes o54 p27)(includes o54 p28)(includes o54 p41)(includes o54 p85)(includes o54 p87)(includes o54 p103)(includes o54 p142)(includes o54 p165)(includes o54 p190)

(waiting o55)
(includes o55 p21)(includes o55 p49)(includes o55 p85)(includes o55 p160)(includes o55 p162)(includes o55 p165)(includes o55 p168)(includes o55 p245)

(waiting o56)
(includes o56 p19)(includes o56 p93)(includes o56 p144)(includes o56 p146)(includes o56 p185)(includes o56 p222)

(waiting o57)
(includes o57 p142)(includes o57 p171)

(waiting o58)
(includes o58 p25)(includes o58 p55)(includes o58 p151)(includes o58 p177)(includes o58 p183)(includes o58 p234)

(waiting o59)
(includes o59 p27)(includes o59 p78)(includes o59 p80)(includes o59 p115)(includes o59 p143)(includes o59 p232)(includes o59 p246)

(waiting o60)
(includes o60 p39)(includes o60 p124)(includes o60 p139)(includes o60 p161)(includes o60 p174)(includes o60 p183)(includes o60 p228)

(waiting o61)
(includes o61 p120)

(waiting o62)
(includes o62 p71)(includes o62 p94)(includes o62 p104)(includes o62 p163)(includes o62 p176)

(waiting o63)
(includes o63 p6)(includes o63 p49)(includes o63 p135)(includes o63 p155)(includes o63 p220)

(waiting o64)
(includes o64 p3)(includes o64 p73)(includes o64 p101)(includes o64 p129)(includes o64 p130)(includes o64 p154)(includes o64 p162)(includes o64 p235)

(waiting o65)
(includes o65 p16)(includes o65 p164)(includes o65 p174)(includes o65 p177)(includes o65 p219)

(waiting o66)
(includes o66 p81)(includes o66 p95)(includes o66 p174)(includes o66 p204)(includes o66 p215)

(waiting o67)
(includes o67 p33)(includes o67 p45)(includes o67 p50)(includes o67 p163)(includes o67 p178)(includes o67 p184)(includes o67 p191)(includes o67 p204)

(waiting o68)
(includes o68 p4)(includes o68 p78)(includes o68 p136)(includes o68 p173)(includes o68 p209)

(waiting o69)
(includes o69 p83)(includes o69 p88)(includes o69 p97)(includes o69 p101)(includes o69 p110)(includes o69 p149)(includes o69 p244)

(waiting o70)
(includes o70 p57)(includes o70 p63)(includes o70 p90)(includes o70 p103)(includes o70 p108)(includes o70 p155)(includes o70 p161)(includes o70 p179)(includes o70 p186)(includes o70 p226)(includes o70 p239)

(waiting o71)
(includes o71 p36)(includes o71 p69)(includes o71 p81)(includes o71 p188)(includes o71 p221)

(waiting o72)
(includes o72 p21)(includes o72 p69)(includes o72 p235)(includes o72 p240)

(waiting o73)
(includes o73 p49)(includes o73 p73)(includes o73 p112)(includes o73 p134)(includes o73 p139)(includes o73 p180)

(waiting o74)
(includes o74 p48)(includes o74 p201)(includes o74 p244)

(waiting o75)
(includes o75 p21)(includes o75 p32)(includes o75 p62)(includes o75 p92)(includes o75 p181)(includes o75 p229)

(waiting o76)
(includes o76 p8)(includes o76 p12)(includes o76 p14)(includes o76 p51)(includes o76 p86)(includes o76 p209)(includes o76 p220)(includes o76 p233)

(waiting o77)
(includes o77 p20)(includes o77 p102)(includes o77 p146)(includes o77 p179)(includes o77 p245)

(waiting o78)
(includes o78 p115)(includes o78 p145)(includes o78 p194)(includes o78 p203)

(waiting o79)
(includes o79 p62)(includes o79 p117)(includes o79 p220)(includes o79 p242)(includes o79 p249)

(waiting o80)
(includes o80 p59)(includes o80 p95)(includes o80 p165)(includes o80 p239)

(waiting o81)
(includes o81 p22)(includes o81 p29)(includes o81 p32)(includes o81 p43)(includes o81 p79)(includes o81 p172)

(waiting o82)
(includes o82 p13)(includes o82 p25)(includes o82 p41)(includes o82 p61)(includes o82 p87)(includes o82 p113)(includes o82 p176)(includes o82 p245)(includes o82 p250)

(waiting o83)
(includes o83 p27)(includes o83 p49)(includes o83 p64)(includes o83 p85)(includes o83 p113)(includes o83 p122)(includes o83 p131)(includes o83 p137)(includes o83 p189)(includes o83 p211)

(waiting o84)
(includes o84 p10)(includes o84 p28)(includes o84 p107)(includes o84 p146)(includes o84 p215)

(waiting o85)
(includes o85 p19)(includes o85 p22)(includes o85 p130)(includes o85 p138)(includes o85 p151)(includes o85 p157)(includes o85 p206)(includes o85 p219)(includes o85 p230)(includes o85 p245)

(waiting o86)
(includes o86 p11)(includes o86 p194)(includes o86 p218)(includes o86 p219)

(waiting o87)
(includes o87 p37)(includes o87 p70)(includes o87 p167)(includes o87 p229)

(waiting o88)
(includes o88 p109)(includes o88 p112)(includes o88 p200)(includes o88 p244)

(waiting o89)
(includes o89 p187)(includes o89 p243)(includes o89 p244)

(waiting o90)
(includes o90 p52)(includes o90 p78)(includes o90 p153)(includes o90 p183)

(waiting o91)
(includes o91 p25)(includes o91 p39)(includes o91 p59)(includes o91 p70)(includes o91 p93)(includes o91 p184)

(waiting o92)
(includes o92 p7)(includes o92 p101)(includes o92 p161)(includes o92 p176)(includes o92 p180)

(waiting o93)
(includes o93 p22)(includes o93 p45)(includes o93 p49)(includes o93 p79)(includes o93 p120)(includes o93 p163)(includes o93 p167)(includes o93 p230)

(waiting o94)
(includes o94 p29)(includes o94 p78)

(waiting o95)
(includes o95 p72)(includes o95 p107)(includes o95 p162)(includes o95 p195)

(waiting o96)
(includes o96 p45)(includes o96 p90)(includes o96 p113)(includes o96 p159)(includes o96 p181)

(waiting o97)
(includes o97 p2)(includes o97 p13)(includes o97 p59)(includes o97 p64)(includes o97 p94)(includes o97 p207)(includes o97 p229)(includes o97 p241)(includes o97 p250)

(waiting o98)
(includes o98 p52)(includes o98 p140)(includes o98 p196)

(waiting o99)
(includes o99 p154)(includes o99 p212)(includes o99 p223)(includes o99 p247)

(waiting o100)
(includes o100 p9)(includes o100 p167)(includes o100 p177)(includes o100 p200)(includes o100 p208)

(waiting o101)
(includes o101 p4)(includes o101 p15)(includes o101 p27)(includes o101 p151)(includes o101 p163)(includes o101 p179)(includes o101 p206)

(waiting o102)
(includes o102 p115)(includes o102 p171)(includes o102 p177)(includes o102 p199)(includes o102 p228)

(waiting o103)
(includes o103 p64)(includes o103 p86)(includes o103 p102)(includes o103 p169)(includes o103 p203)(includes o103 p209)

(waiting o104)
(includes o104 p38)(includes o104 p57)(includes o104 p80)(includes o104 p190)

(waiting o105)
(includes o105 p5)(includes o105 p9)(includes o105 p68)(includes o105 p94)(includes o105 p100)(includes o105 p104)(includes o105 p116)(includes o105 p126)(includes o105 p169)(includes o105 p176)(includes o105 p181)(includes o105 p193)(includes o105 p213)

(waiting o106)
(includes o106 p61)(includes o106 p94)(includes o106 p168)(includes o106 p175)(includes o106 p231)

(waiting o107)
(includes o107 p91)(includes o107 p137)(includes o107 p220)(includes o107 p233)(includes o107 p234)

(waiting o108)
(includes o108 p35)(includes o108 p46)(includes o108 p48)(includes o108 p164)(includes o108 p174)(includes o108 p192)

(waiting o109)
(includes o109 p1)(includes o109 p110)(includes o109 p226)

(waiting o110)
(includes o110 p81)(includes o110 p141)(includes o110 p159)(includes o110 p226)

(waiting o111)
(includes o111 p2)(includes o111 p56)(includes o111 p65)(includes o111 p177)(includes o111 p210)

(waiting o112)
(includes o112 p19)(includes o112 p84)(includes o112 p94)(includes o112 p118)(includes o112 p227)

(waiting o113)
(includes o113 p48)(includes o113 p63)(includes o113 p66)(includes o113 p71)(includes o113 p134)(includes o113 p173)(includes o113 p184)(includes o113 p226)(includes o113 p250)

(waiting o114)
(includes o114 p24)(includes o114 p62)(includes o114 p78)(includes o114 p122)(includes o114 p157)

(waiting o115)
(includes o115 p19)(includes o115 p64)(includes o115 p157)(includes o115 p209)

(waiting o116)
(includes o116 p79)(includes o116 p80)(includes o116 p155)(includes o116 p201)

(waiting o117)
(includes o117 p4)(includes o117 p137)(includes o117 p172)(includes o117 p207)

(waiting o118)
(includes o118 p30)(includes o118 p74)(includes o118 p123)(includes o118 p245)(includes o118 p249)

(waiting o119)
(includes o119 p98)

(waiting o120)
(includes o120 p23)(includes o120 p82)(includes o120 p104)(includes o120 p121)(includes o120 p154)

(waiting o121)
(includes o121 p1)(includes o121 p39)(includes o121 p61)(includes o121 p98)(includes o121 p107)(includes o121 p112)(includes o121 p119)(includes o121 p162)(includes o121 p214)(includes o121 p219)(includes o121 p238)

(waiting o122)
(includes o122 p29)(includes o122 p86)(includes o122 p90)(includes o122 p104)(includes o122 p147)(includes o122 p156)(includes o122 p160)

(waiting o123)
(includes o123 p4)(includes o123 p60)(includes o123 p130)(includes o123 p139)(includes o123 p143)(includes o123 p173)(includes o123 p193)(includes o123 p196)(includes o123 p211)(includes o123 p220)(includes o123 p242)

(waiting o124)
(includes o124 p8)(includes o124 p39)(includes o124 p46)(includes o124 p48)(includes o124 p121)(includes o124 p163)(includes o124 p165)(includes o124 p245)

(waiting o125)
(includes o125 p16)(includes o125 p23)(includes o125 p68)(includes o125 p72)(includes o125 p94)(includes o125 p164)(includes o125 p203)(includes o125 p215)

(waiting o126)
(includes o126 p53)(includes o126 p100)(includes o126 p166)(includes o126 p203)

(waiting o127)
(includes o127 p4)(includes o127 p32)(includes o127 p108)(includes o127 p130)(includes o127 p173)(includes o127 p206)

(waiting o128)
(includes o128 p8)(includes o128 p42)(includes o128 p108)(includes o128 p127)(includes o128 p150)(includes o128 p240)

(waiting o129)
(includes o129 p124)(includes o129 p131)(includes o129 p153)(includes o129 p183)(includes o129 p186)

(waiting o130)
(includes o130 p5)(includes o130 p12)(includes o130 p40)(includes o130 p42)(includes o130 p244)

(waiting o131)
(includes o131 p1)(includes o131 p8)(includes o131 p47)(includes o131 p48)(includes o131 p52)(includes o131 p79)(includes o131 p148)(includes o131 p153)(includes o131 p163)

(waiting o132)
(includes o132 p3)(includes o132 p61)(includes o132 p72)(includes o132 p214)(includes o132 p219)(includes o132 p222)(includes o132 p227)

(waiting o133)
(includes o133 p33)(includes o133 p115)(includes o133 p245)

(waiting o134)
(includes o134 p8)(includes o134 p15)(includes o134 p49)(includes o134 p50)(includes o134 p69)(includes o134 p93)(includes o134 p104)

(waiting o135)
(includes o135 p103)(includes o135 p161)(includes o135 p190)(includes o135 p249)

(waiting o136)
(includes o136 p100)(includes o136 p155)(includes o136 p181)

(waiting o137)
(includes o137 p54)(includes o137 p75)(includes o137 p77)(includes o137 p134)(includes o137 p162)(includes o137 p187)(includes o137 p196)

(waiting o138)
(includes o138 p55)(includes o138 p131)(includes o138 p232)

(waiting o139)
(includes o139 p7)(includes o139 p51)(includes o139 p155)(includes o139 p239)

(waiting o140)
(includes o140 p12)(includes o140 p17)(includes o140 p66)(includes o140 p77)(includes o140 p119)(includes o140 p125)(includes o140 p228)

(waiting o141)
(includes o141 p26)(includes o141 p40)(includes o141 p56)(includes o141 p60)(includes o141 p68)(includes o141 p75)(includes o141 p82)(includes o141 p109)(includes o141 p122)

(waiting o142)
(includes o142 p15)(includes o142 p19)(includes o142 p99)(includes o142 p188)(includes o142 p193)(includes o142 p197)

(waiting o143)
(includes o143 p171)(includes o143 p188)(includes o143 p224)(includes o143 p229)(includes o143 p230)

(waiting o144)
(includes o144 p49)(includes o144 p77)(includes o144 p120)(includes o144 p147)(includes o144 p182)(includes o144 p184)(includes o144 p205)

(waiting o145)
(includes o145 p73)(includes o145 p150)(includes o145 p177)(includes o145 p229)

(waiting o146)
(includes o146 p11)(includes o146 p28)(includes o146 p62)(includes o146 p98)(includes o146 p100)(includes o146 p136)(includes o146 p153)(includes o146 p169)(includes o146 p196)(includes o146 p216)

(waiting o147)
(includes o147 p2)(includes o147 p15)(includes o147 p21)

(waiting o148)
(includes o148 p106)(includes o148 p110)(includes o148 p150)(includes o148 p159)(includes o148 p197)(includes o148 p199)(includes o148 p203)(includes o148 p214)(includes o148 p247)

(waiting o149)
(includes o149 p50)(includes o149 p68)(includes o149 p81)(includes o149 p92)(includes o149 p113)(includes o149 p131)(includes o149 p147)(includes o149 p161)(includes o149 p165)(includes o149 p202)(includes o149 p222)(includes o149 p250)

(waiting o150)
(includes o150 p41)(includes o150 p89)(includes o150 p137)(includes o150 p246)

(waiting o151)
(includes o151 p4)(includes o151 p112)(includes o151 p123)(includes o151 p186)(includes o151 p204)

(waiting o152)
(includes o152 p13)(includes o152 p34)(includes o152 p146)(includes o152 p152)(includes o152 p215)(includes o152 p233)(includes o152 p241)

(waiting o153)
(includes o153 p41)(includes o153 p84)(includes o153 p128)(includes o153 p151)(includes o153 p238)

(waiting o154)
(includes o154 p45)(includes o154 p90)(includes o154 p95)(includes o154 p107)(includes o154 p159)(includes o154 p206)(includes o154 p228)

(waiting o155)
(includes o155 p38)(includes o155 p90)(includes o155 p133)(includes o155 p152)(includes o155 p159)(includes o155 p178)(includes o155 p197)(includes o155 p246)

(waiting o156)
(includes o156 p33)(includes o156 p61)(includes o156 p117)(includes o156 p149)(includes o156 p150)(includes o156 p208)(includes o156 p232)(includes o156 p242)(includes o156 p243)(includes o156 p248)

(waiting o157)
(includes o157 p2)(includes o157 p44)(includes o157 p83)(includes o157 p117)(includes o157 p143)(includes o157 p174)(includes o157 p177)(includes o157 p185)(includes o157 p212)(includes o157 p222)(includes o157 p230)

(waiting o158)
(includes o158 p9)(includes o158 p32)(includes o158 p77)(includes o158 p129)(includes o158 p146)(includes o158 p174)(includes o158 p214)(includes o158 p229)(includes o158 p235)

(waiting o159)
(includes o159 p4)(includes o159 p31)(includes o159 p51)(includes o159 p105)(includes o159 p189)

(waiting o160)
(includes o160 p10)(includes o160 p26)(includes o160 p168)(includes o160 p182)(includes o160 p187)(includes o160 p226)(includes o160 p250)

(waiting o161)
(includes o161 p13)(includes o161 p103)(includes o161 p162)(includes o161 p185)(includes o161 p214)(includes o161 p225)

(waiting o162)
(includes o162 p10)(includes o162 p18)(includes o162 p29)(includes o162 p134)(includes o162 p137)(includes o162 p157)(includes o162 p170)(includes o162 p213)

(waiting o163)
(includes o163 p50)(includes o163 p117)(includes o163 p135)(includes o163 p137)(includes o163 p189)

(waiting o164)
(includes o164 p73)(includes o164 p83)(includes o164 p142)(includes o164 p193)(includes o164 p212)(includes o164 p243)(includes o164 p245)

(waiting o165)
(includes o165 p6)(includes o165 p120)(includes o165 p222)

(waiting o166)
(includes o166 p1)(includes o166 p43)(includes o166 p69)(includes o166 p83)(includes o166 p86)(includes o166 p114)(includes o166 p129)(includes o166 p154)(includes o166 p208)(includes o166 p216)(includes o166 p233)(includes o166 p238)

(waiting o167)
(includes o167 p44)(includes o167 p66)(includes o167 p74)(includes o167 p183)(includes o167 p188)(includes o167 p207)(includes o167 p225)

(waiting o168)
(includes o168 p48)(includes o168 p61)(includes o168 p114)(includes o168 p120)(includes o168 p137)(includes o168 p143)(includes o168 p157)(includes o168 p174)(includes o168 p182)(includes o168 p206)

(waiting o169)
(includes o169 p10)(includes o169 p27)(includes o169 p56)(includes o169 p217)(includes o169 p224)

(waiting o170)
(includes o170 p7)(includes o170 p8)(includes o170 p27)(includes o170 p145)(includes o170 p161)(includes o170 p237)

(waiting o171)
(includes o171 p12)(includes o171 p48)(includes o171 p52)(includes o171 p77)(includes o171 p117)(includes o171 p174)(includes o171 p185)(includes o171 p213)(includes o171 p235)

(waiting o172)
(includes o172 p33)(includes o172 p88)(includes o172 p134)(includes o172 p154)(includes o172 p168)(includes o172 p175)

(waiting o173)
(includes o173 p40)(includes o173 p53)(includes o173 p78)(includes o173 p90)(includes o173 p111)(includes o173 p135)(includes o173 p152)(includes o173 p185)(includes o173 p217)(includes o173 p223)(includes o173 p250)

(waiting o174)
(includes o174 p23)(includes o174 p26)(includes o174 p52)(includes o174 p55)(includes o174 p83)(includes o174 p94)(includes o174 p110)

(waiting o175)
(includes o175 p26)(includes o175 p32)(includes o175 p76)(includes o175 p123)(includes o175 p148)(includes o175 p182)

(waiting o176)
(includes o176 p108)(includes o176 p132)(includes o176 p191)(includes o176 p221)

(waiting o177)
(includes o177 p28)(includes o177 p57)(includes o177 p110)(includes o177 p147)(includes o177 p163)(includes o177 p185)(includes o177 p223)(includes o177 p230)

(waiting o178)
(includes o178 p7)(includes o178 p20)(includes o178 p79)(includes o178 p91)(includes o178 p211)

(waiting o179)
(includes o179 p7)(includes o179 p45)(includes o179 p134)(includes o179 p171)(includes o179 p224)

(waiting o180)
(includes o180 p5)(includes o180 p122)(includes o180 p183)(includes o180 p185)(includes o180 p196)(includes o180 p216)(includes o180 p250)

(waiting o181)
(includes o181 p2)(includes o181 p13)(includes o181 p36)(includes o181 p82)(includes o181 p140)

(waiting o182)
(includes o182 p83)(includes o182 p179)(includes o182 p189)(includes o182 p220)

(waiting o183)
(includes o183 p19)(includes o183 p53)(includes o183 p62)(includes o183 p104)(includes o183 p109)(includes o183 p119)(includes o183 p126)(includes o183 p129)(includes o183 p133)(includes o183 p167)

(waiting o184)
(includes o184 p2)(includes o184 p3)(includes o184 p46)(includes o184 p86)(includes o184 p107)(includes o184 p147)(includes o184 p238)

(waiting o185)
(includes o185 p28)(includes o185 p83)(includes o185 p126)(includes o185 p137)

(waiting o186)
(includes o186 p2)(includes o186 p22)(includes o186 p70)(includes o186 p114)(includes o186 p147)(includes o186 p164)(includes o186 p227)(includes o186 p229)

(waiting o187)
(includes o187 p101)(includes o187 p148)(includes o187 p166)(includes o187 p174)(includes o187 p182)(includes o187 p194)(includes o187 p239)(includes o187 p240)(includes o187 p250)

(waiting o188)
(includes o188 p6)(includes o188 p23)(includes o188 p102)(includes o188 p106)(includes o188 p111)(includes o188 p122)

(waiting o189)
(includes o189 p7)(includes o189 p11)(includes o189 p23)(includes o189 p39)(includes o189 p80)(includes o189 p93)(includes o189 p112)(includes o189 p194)

(waiting o190)
(includes o190 p40)(includes o190 p52)(includes o190 p118)(includes o190 p208)(includes o190 p231)(includes o190 p233)(includes o190 p246)

(waiting o191)
(includes o191 p6)(includes o191 p168)(includes o191 p174)(includes o191 p182)(includes o191 p185)(includes o191 p203)(includes o191 p248)

(waiting o192)
(includes o192 p74)(includes o192 p83)(includes o192 p113)(includes o192 p204)(includes o192 p217)

(waiting o193)
(includes o193 p47)(includes o193 p102)(includes o193 p161)(includes o193 p188)(includes o193 p218)

(waiting o194)
(includes o194 p11)(includes o194 p12)(includes o194 p91)(includes o194 p133)(includes o194 p149)(includes o194 p231)

(waiting o195)
(includes o195 p29)(includes o195 p79)(includes o195 p86)(includes o195 p134)(includes o195 p148)(includes o195 p203)

(waiting o196)
(includes o196 p162)(includes o196 p190)(includes o196 p230)

(waiting o197)
(includes o197 p20)(includes o197 p37)(includes o197 p55)(includes o197 p70)(includes o197 p129)(includes o197 p132)(includes o197 p136)(includes o197 p187)(includes o197 p193)(includes o197 p194)(includes o197 p243)

(waiting o198)
(includes o198 p7)(includes o198 p52)(includes o198 p62)(includes o198 p154)(includes o198 p226)

(waiting o199)
(includes o199 p21)(includes o199 p51)(includes o199 p54)(includes o199 p139)(includes o199 p146)(includes o199 p230)

(waiting o200)
(includes o200 p109)(includes o200 p144)(includes o200 p212)(includes o200 p226)(includes o200 p237)(includes o200 p247)

(waiting o201)
(includes o201 p6)(includes o201 p29)(includes o201 p53)(includes o201 p87)(includes o201 p117)(includes o201 p129)(includes o201 p179)(includes o201 p182)(includes o201 p245)

(waiting o202)
(includes o202 p56)(includes o202 p57)(includes o202 p65)(includes o202 p107)(includes o202 p197)(includes o202 p199)(includes o202 p210)(includes o202 p236)

(waiting o203)
(includes o203 p17)(includes o203 p126)(includes o203 p156)(includes o203 p176)

(waiting o204)
(includes o204 p80)(includes o204 p131)(includes o204 p245)

(waiting o205)
(includes o205 p34)(includes o205 p39)(includes o205 p57)(includes o205 p58)(includes o205 p89)(includes o205 p121)(includes o205 p143)(includes o205 p210)(includes o205 p227)

(waiting o206)
(includes o206 p49)(includes o206 p118)(includes o206 p201)

(waiting o207)
(includes o207 p2)(includes o207 p140)(includes o207 p150)(includes o207 p181)(includes o207 p182)(includes o207 p185)(includes o207 p186)(includes o207 p214)

(waiting o208)
(includes o208 p68)(includes o208 p99)(includes o208 p181)(includes o208 p213)(includes o208 p219)(includes o208 p249)

(waiting o209)
(includes o209 p74)(includes o209 p77)(includes o209 p110)

(waiting o210)
(includes o210 p17)(includes o210 p97)(includes o210 p141)(includes o210 p146)(includes o210 p222)

(waiting o211)
(includes o211 p141)(includes o211 p170)(includes o211 p221)(includes o211 p242)

(waiting o212)
(includes o212 p2)(includes o212 p51)(includes o212 p58)(includes o212 p128)(includes o212 p158)(includes o212 p175)(includes o212 p179)(includes o212 p186)(includes o212 p200)

(waiting o213)
(includes o213 p11)(includes o213 p31)(includes o213 p34)(includes o213 p160)(includes o213 p248)

(waiting o214)
(includes o214 p29)(includes o214 p145)(includes o214 p173)(includes o214 p185)(includes o214 p242)

(waiting o215)
(includes o215 p30)(includes o215 p192)(includes o215 p216)

(waiting o216)
(includes o216 p32)(includes o216 p61)(includes o216 p68)(includes o216 p75)(includes o216 p123)(includes o216 p150)(includes o216 p245)

(waiting o217)
(includes o217 p31)(includes o217 p119)(includes o217 p132)(includes o217 p203)(includes o217 p218)(includes o217 p236)(includes o217 p248)

(waiting o218)
(includes o218 p10)(includes o218 p47)(includes o218 p64)(includes o218 p78)(includes o218 p91)(includes o218 p109)(includes o218 p114)(includes o218 p176)(includes o218 p179)

(waiting o219)
(includes o219 p5)(includes o219 p16)(includes o219 p58)(includes o219 p210)

(waiting o220)
(includes o220 p1)(includes o220 p25)(includes o220 p124)(includes o220 p145)(includes o220 p169)(includes o220 p188)(includes o220 p223)(includes o220 p241)

(waiting o221)
(includes o221 p25)(includes o221 p55)(includes o221 p74)(includes o221 p167)(includes o221 p168)(includes o221 p179)(includes o221 p213)(includes o221 p227)

(waiting o222)
(includes o222 p33)(includes o222 p102)(includes o222 p108)(includes o222 p206)(includes o222 p240)(includes o222 p247)

(waiting o223)
(includes o223 p16)(includes o223 p45)(includes o223 p62)(includes o223 p139)

(waiting o224)
(includes o224 p4)(includes o224 p47)(includes o224 p79)(includes o224 p85)(includes o224 p152)(includes o224 p238)

(waiting o225)
(includes o225 p88)(includes o225 p98)(includes o225 p191)(includes o225 p227)(includes o225 p230)

(waiting o226)
(includes o226 p56)(includes o226 p164)(includes o226 p170)(includes o226 p178)(includes o226 p179)

(waiting o227)
(includes o227 p17)(includes o227 p22)(includes o227 p38)(includes o227 p84)(includes o227 p106)(includes o227 p126)(includes o227 p134)(includes o227 p140)

(waiting o228)
(includes o228 p36)(includes o228 p114)(includes o228 p190)(includes o228 p206)(includes o228 p238)(includes o228 p250)

(waiting o229)
(includes o229 p23)(includes o229 p119)(includes o229 p169)(includes o229 p202)(includes o229 p239)(includes o229 p248)

(waiting o230)
(includes o230 p47)(includes o230 p63)(includes o230 p240)(includes o230 p244)

(waiting o231)
(includes o231 p30)(includes o231 p66)(includes o231 p115)(includes o231 p164)(includes o231 p166)(includes o231 p201)

(waiting o232)
(includes o232 p26)(includes o232 p46)(includes o232 p124)(includes o232 p145)(includes o232 p159)

(waiting o233)
(includes o233 p46)(includes o233 p47)(includes o233 p145)

(waiting o234)
(includes o234 p26)(includes o234 p29)(includes o234 p49)(includes o234 p65)(includes o234 p99)(includes o234 p133)(includes o234 p227)(includes o234 p237)(includes o234 p246)

(waiting o235)
(includes o235 p77)(includes o235 p91)(includes o235 p100)(includes o235 p131)(includes o235 p134)(includes o235 p156)(includes o235 p188)(includes o235 p200)(includes o235 p201)(includes o235 p235)

(waiting o236)
(includes o236 p139)(includes o236 p140)(includes o236 p154)(includes o236 p182)(includes o236 p199)

(waiting o237)
(includes o237 p38)(includes o237 p59)(includes o237 p106)(includes o237 p137)(includes o237 p198)(includes o237 p202)(includes o237 p203)

(waiting o238)
(includes o238 p25)(includes o238 p63)(includes o238 p136)(includes o238 p155)(includes o238 p159)(includes o238 p195)

(waiting o239)
(includes o239 p8)(includes o239 p48)(includes o239 p180)(includes o239 p186)

(waiting o240)
(includes o240 p123)(includes o240 p137)(includes o240 p143)(includes o240 p227)

(waiting o241)
(includes o241 p4)(includes o241 p21)(includes o241 p205)(includes o241 p221)(includes o241 p235)(includes o241 p240)

(waiting o242)
(includes o242 p79)(includes o242 p101)(includes o242 p154)(includes o242 p209)(includes o242 p245)

(waiting o243)
(includes o243 p10)(includes o243 p24)(includes o243 p41)(includes o243 p86)(includes o243 p152)(includes o243 p156)

(waiting o244)
(includes o244 p20)(includes o244 p54)(includes o244 p80)(includes o244 p160)(includes o244 p174)(includes o244 p190)(includes o244 p214)(includes o244 p226)

(waiting o245)
(includes o245 p94)(includes o245 p98)(includes o245 p183)(includes o245 p210)(includes o245 p211)

(waiting o246)
(includes o246 p16)(includes o246 p25)(includes o246 p30)(includes o246 p86)(includes o246 p107)(includes o246 p109)(includes o246 p134)(includes o246 p142)(includes o246 p184)(includes o246 p219)

(waiting o247)
(includes o247 p12)(includes o247 p32)(includes o247 p79)(includes o247 p81)(includes o247 p114)(includes o247 p127)(includes o247 p134)(includes o247 p195)(includes o247 p226)(includes o247 p245)

(waiting o248)
(includes o248 p26)(includes o248 p73)(includes o248 p112)(includes o248 p182)(includes o248 p217)(includes o248 p222)(includes o248 p229)

(waiting o249)
(includes o249 p53)(includes o249 p108)

(waiting o250)
(includes o250 p102)(includes o250 p192)(includes o250 p202)(includes o250 p234)(includes o250 p248)

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
(not-made p211)
(not-made p212)
(not-made p213)
(not-made p214)
(not-made p215)
(not-made p216)
(not-made p217)
(not-made p218)
(not-made p219)
(not-made p220)
(not-made p221)
(not-made p222)
(not-made p223)
(not-made p224)
(not-made p225)
(not-made p226)
(not-made p227)
(not-made p228)
(not-made p229)
(not-made p230)
(not-made p231)
(not-made p232)
(not-made p233)
(not-made p234)
(not-made p235)
(not-made p236)
(not-made p237)
(not-made p238)
(not-made p239)
(not-made p240)
(not-made p241)
(not-made p242)
(not-made p243)
(not-made p244)
(not-made p245)
(not-made p246)
(not-made p247)
(not-made p248)
(not-made p249)
(not-made p250)

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
(shipped o211)
(shipped o212)
(shipped o213)
(shipped o214)
(shipped o215)
(shipped o216)
(shipped o217)
(shipped o218)
(shipped o219)
(shipped o220)
(shipped o221)
(shipped o222)
(shipped o223)
(shipped o224)
(shipped o225)
(shipped o226)
(shipped o227)
(shipped o228)
(shipped o229)
(shipped o230)
(shipped o231)
(shipped o232)
(shipped o233)
(shipped o234)
(shipped o235)
(shipped o236)
(shipped o237)
(shipped o238)
(shipped o239)
(shipped o240)
(shipped o241)
(shipped o242)
(shipped o243)
(shipped o244)
(shipped o245)
(shipped o246)
(shipped o247)
(shipped o248)
(shipped o249)
(shipped o250)
))

(:metric minimize (total-cost))

)


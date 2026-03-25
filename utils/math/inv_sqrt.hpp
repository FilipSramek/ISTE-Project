#pragma once

namespace math
{
    /**
     * @brief Fast inverse square root approximation using the Quake III method
     * @param x Input value (must be positive)
     * @return Approximation of 1/sqrt(x)
     */
    inline float inv_sqrt(float x)
    {
        long i;
        float xHalf;
        float y;
        const float threehalfs = 1.5f;

        xHalf = x * 0.5f;       //vydělím x dvěma a uložím do xHalf, protože se bude používat později v korekčním kroku
        y = x;

        
        i = * (long *) &y;      // přetypování float na long tak aby mi zůstala řada bitů stejná, ale interpretace se změnila z float na long (klasické přetypování by zaokrouhlilo číslo na int)
                                // vezmu adresu floutu a změním ji na adresu longu, poté tuto adresu dereferencuji a získám long reprezentaci bitů floatu
                                // přesto, že je to to long, tak v datech je pořád exponent a mantisa 
        
        i = 0x5f3759df - (i >> 1);   //0x5f3759df je konstanta, která využíva vlastnosti logratimu (dá se dobře lineárně aproximovat mezi 0 a 1) 
                                     //díky této aporoximaci můžu řít pofiderní větu: y = log(y), nebo přesnější: y + C = log(y) 
                                     //díky tomu můžu hodně ulehčit výpočet. log(1/sqrt(x)) = log(x^(-1/2)) = -1/2 * log(x) 
                                     //dál využiju aproximaci a dostanu: -1/2 * x + C
                                     //dělení můžu vypočítat pomocí bit shiftu, protože dělení dvěma je posun bitů o 1 doprava (i >> 1)

        y = * (float *) &i;     // opět přetypování longu zpět na float, takže teď mám v y přibližnou hodnotu 1/sqrt(x)
                                // není to špatná aproximace, ale není skvělá
                                // dostane nás velmi rychle velmi blízo a už jen pár iterací Newtonovy metody nám dá velmi přesný výsledek

        y = y * (threehalfs - (xHalf * y * y));   // Newtonova metoda pro hledání odmocniny
                                                  // hledá kořen funkce f(y) pomocí iterativního procesu přes podíl derivací: y = y - f(y) / f'(y),
                                                  // kde f(y) = 1/y^2 - x, takže f'(y) = -2/y^3 
                                                  // po dosazení do vzorce dostaneme: y = y - (1/y^2 - x) / (-2/y^3) = y + (1/y^2 - x) * (y^3 / 2) = y + (y/2) * (1 - x*y^2)
                                                  // což můžeme přepsat na: y = y * (3/2 - (x/2)*y^2) = y * (threehalfs - (xHalf * y * y))
                                                  // tímto jsem se v celém algoritmu vyhnul dělení.
                                                  // první iterace nás dostane na přesnost kolem 1% od skutečné hodnoty
        y = y * (threehalfs - (xHalf * y * y));   // druhá iterace nás dostane na přesnost kolem 0.0001% od skutečné hodnoty (to stačí)
        return y;
    }
}
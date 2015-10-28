#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <vector>

#include "individual.h"

Individual createTestIndividual(const int nbFares, const int nbRoutes) {
    Fare tmp;

    tmp.journeyId = 1;
    tmp.plannedElementIdx = 2;
    tmp.crewIdx = 3;
    tmp.timestamp = 4;
    tmp.typeVehicle = 5;
    tmp.price = 6;
    tmp.realTimestamp = 7;
    tmp.employeeIdx[0] = 8;
    tmp.employeeIdx[1] = 9;
    tmp.crewSize = 10;

    Individual tmpIndiv;

    std::fill_n(tmpIndiv.listFare.begin(), nbRoutes * nbFares, tmp);
    std::fill_n(tmpIndiv.vehiclesCount.begin(), nbRoutes, nbFares);

    return tmpIndiv;
}

int vehicleResponsible(Individual& indiv, int pos) {
    int count = 0;
    int i = 0;

    while (pos >= count ) {
        count += indiv.vehiclesCount[i];
        i++;
    }

    return i - 1;
}

int checkDominanceHost(const Fitness& ind1, const Fitness& ind2, int nobj) {
    bool isBetter = false;
    bool isWorse = false;

    for (int i = 0; i < nobj; i++) {
        if (ind1[i] <= ind2[i]) {
            isBetter = true;

        } else {
            if (ind1[i] > ind2[i]) {
                isWorse = true;
            }
        }
    }
    if (isBetter && (!isWorse)) {
        return 1;
    } else if ((!isBetter) && isWorse) {
        return 0;
    } else {
        return (-1);
    }

}

void toTail(array<FareVector, MAX_NB_VEHICLE>& sIndiv, int src, int dest,
            int offset) {
    for (int j = 0; j < sIndiv[src] - sIndiv[dest]; j++) {
        insertJourneyElements(sIndiv[dest], sIndiv[src][offset + j]);
    }

    sIndiv[v1].size = sIndiv[v2].size;
}

void swapEdge(Individual& indiv, Individual& result, int v1, int v2,
              int pos1, int pos2) {
    array<FareVector, MAX_NB_VEHICLE> sIndiv;
    splitIndividual(indiv, sIndiv);

    int i;

    for (i = 0; i < std::min(sIndiv[v1].size, sIndiv[v2].size); i++) {
        std::swap(sIndiv[v1][pos1 + i], sIndiv[v2][pos2 + i]);
    }

    if(sIndiv[v1].size > sIndiv[v2].size) {
        toTail(sIndiv, v1, v2, i);
    } else if (sIndiv[v1].size < sIndiv[v2].size) {
        toTail(sIndiv, v2, v1, i);
    }

    stitchIndividual(result, sIndiv);
}

void twoOptStar() {
    bool improvement = true;

    while (improvement) {
        for(size_t v = 0; v < std::min(indiv.vehiclesCount.size(), maxNbVehicle); ++v) {
            for(int i = pos; i < pos + indiv.vehiclesCount[v]; i++) {
                /* Only exchange adjacent plannedElement */
                if ((fares[i].journeyId != fares[i + 1].journeyId)
                        && (fares[i + 2].journeyId != fares[i + 3].journeyId))
                    continue;

                // Compute old cost of the two route from the changing point
                //from i

                Fitness base;

                base[COST]  = costFitness(&fares[i],
                                          (vehiclesCount[v] + pos) - i,
                                          data,
                                          costPerKm,
                                          cube,
                                          vehicleIdx,
                                          indiv,
                                          options,
                                          NORMAL);

                base[DELAY] = delayFitness(&fares[i],
                                           (vehiclesCount[v] + pos) - i,
                                           data,
                                           costPerKm,
                                           cube,
                                           vehicleIdx,
                                           indiv,
                                           options,
                                           NORMAL);

                base[LOAD]  = loadFitness(&fares[i],
                                          (vehiclesCount[v] + pos) - i,
                                          data,
                                          costPerKm,
                                          cube,
                                          vehicleIdx,
                                          indiv,
                                          options,
                                          NORMAL);

                for(size_t v2 = 0; v2 < std::min(indiv.vehiclesCount.size(), maxNbVehicle);
                        ++v2) {

                    // Different vehicle type
                    if (data.vehicle(v2).type != data.vehicle(v).type) {
                        continue;
                    }

                    for(int j = pos2; j < pos2 + indiv.vehiclesCount[v2]; j++) {

                        if ((vehiclesCount[v2] + pos2) - j < 4) {
                            break;
                        }

                        /* Only exchange adjacent plannedElement */
                        if ((fares[j].journeyId != fares[j + 1].journeyId)
                                && (fares[j + 2].journeyId != fares[j + 3].journeyId))
                            continue;

                        // Time window
                        if (fares[j + 2].realTimestamp < fares[i + 1].realTimestamp) {
                            continue;
                        }


                        Fitness newGC;
                        Fitness baseCpy = base;
                        //from j
                        baseCpy[DELAY] += delayFitness(&fares[j],
                                                       (vehiclesCount[v2] + pos2) - j,
                                                       data,
                                                       costPerKm,
                                                       cube,
                                                       vehicleIdx,
                                                       indiv,
                                                       options,
                                                       NORMAL);

                        baseCpy[COST]  += costFitness(&fares[j],
                                                      (vehiclesCount[v2] + pos2) - j,
                                                      data,
                                                      costPerKm,
                                                      cube,
                                                      vehicleIdx,
                                                      indiv,
                                                      options,
                                                      NORMAL);

                        baseCpy[LOAD]  += loadFitness(&fares[j],
                                                      (vehiclesCount[v2] + pos2) - j,
                                                      data,
                                                      costPerKm,
                                                      cube,
                                                      vehicleIdx,
                                                      indiv,
                                                      options,
                                                      NORMAL);

                        Individual tmp(data.plannedElements().size() + 1);

                        //switch
                        swapEdge(indiv, tmp, v, v2, i + 1, j + 1):
                            sortFaresByTimestamp(tmp, j, (vehiclesCount[v2] + pos2) - j );
                        sortFaresByTimestamp(tmp, i, (vehiclesCount[v] + pos) - i);

                        newValue[DELAY] = delayFitness();
                        newValue[COST] = costFitness();
                        newValue[LOAD] = loadFitness();
                        // and j
                        newValue[DELAY] += delayFitness();
                        newValue[COST] += costFitness();
                        newValue[LOAD] += loadFitness();

                        auto res = checkDominanceHost(newValue, baseCpy, 3);

                        if (res == 1) {
                            indiv = tmp;
                        }

                        // Compute old cost of the two route
                        // Compute old delay
                        // Compute old capacity


                    }
                    pos2 += indiv.vehiclesCount[v2];
                }
            }
        }
        pos += indiv.vehiclesCount[v];
    }
}

int main(int argc, const char *argv[]) {

    return 0;
}

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

    /* Swap the until the end of the shortest route */
    for (i = 0; i < std::min(sIndiv[v1].size, sIndiv[v2].size); i++) {
        std::swap(sIndiv[v1][pos1 + i], sIndiv[v2][pos2 + i]);
    }

    /* Move the remaining tail to the other route */
    if(sIndiv[v1].size > sIndiv[v2].size) {
        toTail(sIndiv, v1, v2, i);
    } else if (sIndiv[v1].size < sIndiv[v2].size) {
        toTail(sIndiv, v2, v1, i);
    }

    stitchIndividual(result, sIndiv);
}

Fitness evaluateRouteFromPoint(Individual& indiv,
                               const cube_matrix& cube,
                               const Options& options,
                               int offset,
                               size_t remainingElement,
                               int costPerKm,
                               int vehicleIdx) {
    Fitness base;

    base[COST] = costFitness(&indiv.listFare[offset], remainingElement, data,
                             costPerKm, cube, vehicleIdx, indiv, options, NORMAL);
    base[DELAY] = delayFitness(&indiv.listFare[offset], remainingElement, data,
                               costPerKm, cube, vehicleIdx, indiv, options, NORMAL);
    base[LOAD] = loadFitness(&indiv.listFare[offset], remainingElement, data,
                             costPerKm, costPerKm, cube, vehicleIdx, indiv,options, NORMAL);

    return base;
}

void twoOptStar(Individual& indiv, const Data& data, const Options& options) {
    bool improvement = true;

    while (improvement) {
        improvement = false;

        /* Iterate through the vehicle as base vehicle */
        for(size_t v = 0; v < std::min(indiv.vehiclesCount.size(), maxNbVehicle); ++v) {
            auto vehicleI = data.vehicle(v);

            /* Iterate through the edges of the base vehicle*/
            for(int i = offsetBase; i < offsetBase + indiv.vehiclesCount[v]; i++) {
                auto fares& = indiv.listFares;

                /* Only exchange adjacent plannedElement */
                if ((fares[i].journeyId != fares[i + 1].journeyId)
                        && (fares[i + 2].journeyId != fares[i + 3].journeyId))
                    continue;

                /* Compute old cost of the base route from the changing point */
                Fitness base;
                base = evaluateRouteFromPoint(indiv, cube, options, i,
                                              (vehiclesCount[v] + offsetBase) - i, vehicleI.costPerKm, v);

                /* Iterate through the vehicles as the second one */
                for(size_t v2 = 0; v2 < std::min(indiv.vehiclesCount.size(), maxNbVehicle);
                        ++v2) {

                    auto vehicleII = data.vehicle(v2);

                    /* Different vehicle type */
                    if (vehicleI.type != vehicleII.type) {
                        continue;
                    }

                    /* Same vehicle*/
                    if (v != v2) {
                        continue;
                    }

                    for(int j = offset; j < offset + indiv.vehiclesCount[v2]; j++) {

                        /* Not enough plannedElement in the route to attempt local
                         * optimisation */
                        if ((vehiclesCount[v2] + offset) - j < 4) {
                            break;
                        }

                        /* Only exchange adjacent plannedElement: the pick-up
                         * element must be next to the delivery */
                        if ((fares[j].journeyId != fares[j + 1].journeyId)
                                && (fares[j + 2].journeyId != fares[j + 3].journeyId))
                            continue;

                        /* At this point in the vehicle the temporal order must
                         * respected */
                        if (fares[j + 2].realTimestamp < fares[i + 1].realTimestamp) {
                            continue;
                        }


                        /* Compute old cost of the second route from its changing point */
                        Fitness oldValue = evaluateRouteFromPoint(indiv,
                                           cube,
                                           options,
                                           j,
                                           (vehiclesCount[v2] + offset) - j,
                                           vehicleII.costPerKm,
                                           v2);

                        /* Sum the old partial routes cost */
                        for (int obj = 0; obj < NSGA2_OBJ; obj++) {
                            oldValue[obj] += base[obj];
                        }

                        /* Create temporary individual to compute the edge switching*/
                        Individual tmp(data.plannedElements().size() + 1);

                        /* Switch the edges */
                        swapEdge(indiv, tmp, v, v2, i + 1, j + 1);
                        sortFaresByTimestamp(tmp, j, (vehiclesCount[v2] + offset) - j );
                        sortFaresByTimestamp(tmp, i, (vehiclesCount[v] + offsetBase) - i);

                        /* Compute the new cost of the base route from its changing point */
                        auto tmpVecBase = vehicleResponsible(tmp, i);
                        Fitness newValueBase = evaluateRouteFromPoint(tmp, cube, options, i,
                                               (vehiclesCount[tmpVecBase] + offset) - i, vehicleI.costPerKm, v);

                        /* Compute the new cost of the second route from its changing point */
                        auto tmpVec = vehicleResponsible(tmp, j);
                        Fitness newValue = evaluateRouteFromPoint(tmp, cube, options, j,
                                           (vehiclesCount[tmpVec] + offset) - j, vehicleII.costPerKm, v2);

                        /* Sum the new partial routes cost */
                        for (int obj = 0; obj < NSGAII_OBJ; obj++) {
                            newValue[obj] += newValueBase[obj];
                        }

                        /* Compare the sums of the old and new partial cost route */
                        auto res = checkDominanceHost(newValue, oldValue, NSGA2_OBJ);

                        if (res == 1) {
                            indiv = tmp;
                            improvement = true;
                        }

                    }
                    offset += indiv.vehiclesCount[v2];
                }
            }
        }
        offsetBase += indiv.vehiclesCount[v];
    }
}

int main(int argc, const char *argv[]) {

    return 0;
}

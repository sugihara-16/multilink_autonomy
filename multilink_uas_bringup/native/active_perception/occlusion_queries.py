"""Exact angular-neighborhood lookup for the planner's existing occlusion rule."""
import numpy as np
from scipy.spatial import cKDTree


def unoccluded(directions,distances,eligible,rays,ranges,cosine,margin):
    result=eligible.copy();indices=np.flatnonzero(eligible)
    if not len(rays) or not len(indices):return result
    # Enlarging the search by roundoff only finds extra candidates; the
    # original strict dot-product test below remains the final predicate.
    radius=np.sqrt(max(0.,2.-2.*cosine))+1e-10
    nearby=cKDTree(rays).query_ball_point(directions[indices],radius)
    for i,neighbors in zip(indices,nearby):
        if not neighbors:continue
        ids=np.asarray(neighbors);aligned=rays[ids]@directions[i]>cosine
        if np.any(ranges[ids[aligned]]+margin<distances[i]):result[i]=False
    return result

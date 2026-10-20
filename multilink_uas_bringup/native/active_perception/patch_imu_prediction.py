"""Generate an isolated ROVIO prediction header with optional sensor bias dynamics."""
def patch(source):
    assert source.count('const V3D g_;')==1
    source=source.replace('const V3D g_;','V3D g_;')
    source='#include "imu_bias_model.hpp"\n'+source
    for state,tau in [('acb','acc_tau'),('gyb','gyro_tau')]:
        old=f'output.{state}() = state.{state}()+noise.template get<mtNoise::_{state}>()*sqrt(dt);'
        new=f'output.{state}() = active_imu::decay(active_imu::{tau}(),dt).cwiseProduct(state.{state}())+active_imu::noise_scale(active_imu::{tau}(),dt).cwiseProduct(noise.template get<mtNoise::_{state}>());'
        assert source.count(old)==1,old
        source=source.replace(old,new)
        old=f'F.template block<3,3>(mtState::template getId<mtState::_{state}>(),mtState::template getId<mtState::_{state}>()) = M3D::Identity();'
        new=old.replace('M3D::Identity()',f'active_imu::decay(active_imu::{tau}(),dt).asDiagonal()')
        assert source.count(old)==1,old
        source=source.replace(old,new)
        old=f'G.template block<3,3>(mtState::template getId<mtState::_{state}>(),mtNoise::template getId<mtNoise::_{state}>()) = M3D::Identity()*sqrt(dt);'
        new=old.replace('M3D::Identity()*sqrt(dt)',f'active_imu::noise_scale(active_imu::{tau}(),dt).asDiagonal()')
        assert source.count(old)==1,old
        source=source.replace(old,new)
    return source

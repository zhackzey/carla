// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <carla/PythonUtil.h>
#include <carla/client/ClientSideSensor.h>
#include <carla/client/GnssSensor.h>
#include <carla/client/IMUSensor.h>
#include <carla/client/LaneInvasionSensor.h>
#include <carla/client/Sensor.h>
#include <carla/client/ServerSideSensor.h>

#include <carla/client/RandomEngine.h>

static void SubscribeToStream(carla::client::Sensor &self, boost::python::object callback) {
  self.Listen(MakeCallback(std::move(callback)));
}

static void SetNoiseFunction(carla::client::IMUSensor &sensor, boost::python::object func) {
  // // Make sure the callback is actually callable.
  if (!PyCallable_Check(func.ptr())) {
    PyErr_SetString(PyExc_TypeError, "callback argument must be callable!");
    boost::python::throw_error_already_set();
  }

  // We need to delete the callback while holding the GIL.
  using Deleter = carla::PythonUtil::AcquireGILDeleter;
  auto func_ptr = carla::SharedPtr<boost::python::object>{new boost::python::object(func), Deleter()};

  std::function<float(void)> noise = [noise_func=std::move(func_ptr)]() -> float {
    try {
      carla::PythonUtil::AcquireGIL lock;
      return boost::python::call<float>(noise_func->ptr(), boost::python::object());
    } catch (const boost::python::error_already_set &) {
      PyErr_Print();
      return 0.0f;
    }
  };
  sensor.SetNoiseFunction(std::move(noise));
}


void export_sensor() {
  using namespace boost::python;
  namespace cc = carla::client;

  class_<cc::Sensor, bases<cc::Actor>, boost::noncopyable, boost::shared_ptr<cc::Sensor>>("Sensor", no_init)
    .add_property("is_listening", &cc::Sensor::IsListening)
    .def("listen", &SubscribeToStream, (arg("callback")))
    .def("stop", &cc::Sensor::Stop)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cc::ServerSideSensor, bases<cc::Sensor>, boost::noncopyable, boost::shared_ptr<cc::ServerSideSensor>>
      ("ServerSideSensor", no_init)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cc::ClientSideSensor, bases<cc::Sensor>, boost::noncopyable, boost::shared_ptr<cc::ClientSideSensor>>
      ("ClientSideSensor", no_init)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cc::LaneInvasionSensor, bases<cc::ClientSideSensor>, boost::noncopyable, boost::shared_ptr<cc::LaneInvasionSensor>>
      ("LaneInvasionSensor", no_init)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cc::IMUSensor, bases<cc::Sensor>, boost::noncopyable, boost::shared_ptr<cc::IMUSensor>>
      ("IMUSensor", no_init)
    .def(self_ns::str(self_ns::self))
    .def_readwrite("bias", &cc::IMUSensor::bias)
    .def("set_default_noise", &cc::IMUSensor::SetDefaultNoise, (arg("mean"), arg("stddev")))
    .def("set_custom_noise", &SetNoiseFunction, (arg("func")))
  ;

  class_<cc::GnssSensor, bases<cc::Sensor>, boost::noncopyable, boost::shared_ptr<cc::GnssSensor>>
      ("GnssSensor", no_init)
    .def(self_ns::str(self_ns::self))
  ;
}

#ifndef PROCEDURES_HPP
#define PROCEDURES_HPP

/*
  This class provides definitions of the different
  functors that are responsible for the various movement
  and feedback patterns that polaris must execute.
 */

#include "navigation/nav.h"
#include "navigation/nav_request.h"
#include "vision/vector.h"
#include "vision/change_detection.h"

namespace procedures {
  class Procedure {
  public:

    // This is used to communicate what the return code means
    // to the caller.
    enum class ReturnCode : int {
      FATAL = -2,
      ERROR = -1,
      CONTINUE = 0,
      NEXT = 1
    };

    // Initialize class members and load the movement
    // configuration.
    Procedure() = default;

    // Member for making copies of functors.
    virtual Procedure* clone() const
	  {
		  return new Procedure(*this);
	  }

		// Functors require access to sensors.
		// And mechanism to control submarine.   
 
    // Invoking base operator() will result in FATAL
    // return code.
    virtual ReturnCode operator()() {
			// Sensors to measure actual
			// Compare actual to goal.
			// HMMM this sounds like a PID controller...
			// Could this be provided for convenience in the base 
			// class? or should it be left up to the derived?

			// Adjust movement based on previous computation.

			// Based on computation return code to tell StateMachine
			// what the state should be doing.

			return ReturnCode::FATAL;
    };


		// This function is responsible for preparing
    // the sub and functor for use.
		virtual void prep() {
		}

		// This function is repsonsible for unpreparing
    // the sub and functor for use.
		virtual void unprep() {
		}
  };

  class DiveProcedure : public Procedure {
  public:
    DiveProcedure() = default;

    DiveProcedure* clone() const override
    {
    	return new DiveProcedure(*this);
    }

    Procedure::ReturnCode operator()() override
    {
      // Monitor depth of submarine.

      // Adjust depth of submarine.

      // Return status.
      return Procedure::ReturnCode::CONTINUE;
    }
  };

  class IdleProcedure : public Procedure {
	  ros::NodeHandle n;
	  ros::ServiceClient set_heading;

  public:
	  IdleProcedure()
	  : n{}, set_heading(n.serviceClient<navigation::nav_request>("/navigation/set_heading"))
	  {}
  	IdleProcedure* clone() const override
	  {
  		return new IdleProcedure(*this);
	  }

	  Procedure::ReturnCode operator()() override {
		  navigation::nav_request srv;

		  srv.request.depth = 5;
		  srv.request.yaw_rate = 0;
		  srv.request.forwards_velocity = 0;
		  srv.request.sideways_velocity = 0;

		  if (!set_heading.call(srv))
		  {
			  ROS_WARN("Heading service call failed.");
		  }

		  return Procedure::ReturnCode::CONTINUE;
	  }
  };

  class SurfaceProcedure : public Procedure {
		ros::NodeHandle n;
		ros::ServiceClient set_heading;

	public:
		SurfaceProcedure()
		: n{}, set_heading(n.serviceClient<navigation::nav_request>("/navigation/set_heading"))
		{}
		SurfaceProcedure* clone() const override
		{
			return new SurfaceProcedure(*this);
		}

		Procedure::ReturnCode operator()() override {
			navigation::nav_request srv;

			srv.request.depth = 0;
			srv.request.yaw_rate = 0;
			srv.request.forwards_velocity = 0;
			srv.request.sideways_velocity = 0;

			if (!set_heading.call(srv))
			{
				ROS_WARN("Heading service call failed.");
			}

			return Procedure::ReturnCode::CONTINUE;
		}
	};


  class ProcedureA : public Procedure {
	  std::size_t iterations;

  public:
	  ProcedureA() = default;

    ProcedureA* clone() const override
	  {
		  return new ProcedureA(*this);
	  }

	  Procedure::ReturnCode operator()() override
	  {
		  if(iterations == 5)
			  return Procedure::ReturnCode::NEXT;
		  ++iterations;
		  ROS_INFO_STREAM("ProcedureA");
		  return Procedure::ReturnCode::CONTINUE;
	  }
  };

	class ProcedureB : public Procedure {
		std::size_t iterations;

	public:
		ProcedureB() = default;

		ProcedureB* clone() const override
		{
			return new ProcedureB(*this);
		}

		Procedure::ReturnCode operator()() override
		{
			if(iterations == 5)
				return Procedure::ReturnCode::NEXT;
			++iterations;
			ROS_INFO_STREAM("ProcedureB");
			return Procedure::ReturnCode::CONTINUE;
		}
	};

}

#endif

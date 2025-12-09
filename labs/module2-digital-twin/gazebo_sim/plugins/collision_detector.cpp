#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class CollisionDetector : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the world
      this->world = _parent;

      // Listen to the update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CollisionDetector::OnUpdate, this));

      gzdbg << "CollisionDetector plugin loaded!\n";
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Check for collisions between all contactable entities
      physics::ContactManager *mgr = this->world->Physics()->GetContactManager();
      mgr->UpdateContacts();

      // Get all contacts
      const physics::ContactManager::ContactMap contacts = mgr->GetContacts();

      // Process contacts
      for (const auto &contact : contacts)
      {
        if (contact.second->count > 0)
        {
          // Print collision information
          std::string collision_info =
            "Collision between " +
            contact.second->collision1->GetScopedName() +
            " and " +
            contact.second->collision2->GetScopedName();

          // In a real implementation, you would publish this information
          // to a ROS topic for the robot to process
          gzdbg << collision_info << "\n";
        }
      }
    }

    // Pointer to the world
    private: physics::WorldPtr world;

    // Connection to the world update event
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(CollisionDetector)
}
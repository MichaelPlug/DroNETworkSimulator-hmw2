"""
PER CHI LEGGERÀ: NON SO COME IMPORTARE DA SRC.UTILITIES.CONFIG IL NUMERO 
DI DRONI CHE SONO INIZIALIZZATI LÀ
"""

from operator import ne
import numpy as np
import math
from numpy.core.defchararray import array 
from numpy.core.numeric import NaN
from numpy.lib.type_check import real_if_close
from src.routing_algorithms.georouting import GeoRouting
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from matplotlib import pyplot as plt


#TO FIX TO OBTAIN THE NUMBER OF DRONES (COSTANT OF CONFIG.PY)
import src.utilities.config as config #try self.simulator.n_drones

#GLOBAL THINGS

#import the library for random values
import random



"""##!!
##FOR NORMAL REINFORCEMENT LEARNING AND NORMAL Q ARRAY
#create dictionaries (because they are more indicated)
for i in range(5):
    
    #we assume to have optimistic initial value as strategy for action selection
    q[i] = 2
    
    #initially we have zero attempts for each element
    n[i] = 0
##END FOR NORMAL REINFORCEMENT LEARNING AND NORMAL Q ARRAY
"""   
    
#seed for random values, just to have consistence on values 
#TODO
#eliminate after a while
random.seed(2)

#epsilon must be smaller and it represents probability for epsilon-greedy
second_epsilon = 0.05
min_epsilon = 0.05
max_epsilon = 0.25  

georouting_on_next_step = True

#take a random value between 0,1
epsilon = random.random()

#normalize the random value from min_epsilon to max_epsilon
epsilon = min_epsilon + (epsilon * (max_epsilon - min_epsilon))



class AIRouting(BASE_routing):
    
    
    
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        self.taken_actions = {}  #id event : (old_action)
        
        
        

    def feedback(self, drone, id_event, delay, outcome):
        """ return a possible feedback, if the destination drone has received the packet """
        # Packets that we delivered and still need a feedback
        #print(self.drone.identifier, "----------", self.taken_actions)

        # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
        # Feedback from a delivered or expired packet
        #print(self.drone.identifier, "----------", drone, id_event, delay, outcome)
      
        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple feedback for the same packet!!
        # NOTE: reward or update using the old action!!
        # STORE WHICH ACTION DID YOU TAKE IN THE PAST.
        # do something or train the model (?)
        
        print("SONO QUIII")
        
        #if the packet isn't still treated, then we train system for it
        if True:
        
            "Doubt: i don't know the utility of this"        
            if id_event in self.taken_actions:
                action = self.taken_actions[id_event]
                del self.taken_actions[id_event]
            "End of doubt"
                
            #if the packet is arrived isn't more valid
            if (outcome == -1):
                
                #we obtain a small reward 
                R = -1
            
                #R = -2
            
            #if the packet arrived
            else:
                          
                #opposite of delay
                temp = 2000 - delay
                
                #we obtain a linear reward based on the delay --> 
                #more the delay and value more close value to 1... 
                #less the delay and value more close to 2 
                temp = (temp - 0) / (2000 - 0)
                #take the reward
                R = 1 + temp 
              
                
                #R = 2 * (1 - delay/self.simulator.event_duration)
                
            #add attempts for the starting drone that has initially the packet
            #TODO
            #maybe also for all the path of packets to incentive themù

   
            
            try:
                drone_iden = drone.Reward[id_event]
                        
            except Exception as e:
                
                drone_iden = drone
                
            try:
            	n = self.drone.n
            except:
                setattr(self.drone, "n", {})
                
            try:
            	q = self.drone.q
            except:
                setattr(self.drone, "q", {})
                
            try:
                self.drone.n[(drone_iden.identifier,drone_iden.next_target())] += 1
                self.drone.q[(drone_iden.identifier,drone_iden.next_target())] = q[(drone_iden.identifier,drone_iden.next_target())] + ((1/(n[(drone_iden.identifier,drone_iden.next_target())]))*(R - q[(drone_iden.identifier,drone_iden.next_target())]))
            except Exception as e:
                self.drone.n[(drone_iden.identifier,drone_iden.next_target())] = 1
                self.drone.q[(drone_iden.identifier,drone_iden.next_target())] = R #0


            
            
            
            
            
    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """

        """ arg min score  -> geographical approach, take the drone closest to the depot """
        
        #we take our distance from the depot
        best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
       # best_drone_distance_from_depot = self.compute_distance_to_trajectory_s()
        #initially drone closest is us (we take to the depot the
        #packet without any help)
        best_drone = None
        
        
        if (len(opt_neighbors) == 0):
            
            return -1
        
        
        #generate a random value between 0 and 1
        rand = random.random()
        
                
        try:
           q = self.drone.q
           n = self.drone.n
        except:
           setattr(self.drone, "q", {})
           q = self.drone.q
           setattr(self.drone, "n", {})
           n = self.drone.n

        

        q_distance = best_drone_distance_from_depot
    


        a = True
        if a:
     #   if (rand < (1-newEps)):
            
            try:
            
                #take the maximum value of q
                max_q = q[(self.drone.identifier,self.drone.next_target())]
            
            except Exception as e:
                
                q[(self.drone.identifier,self.drone.next_target())] = 0
                
                max_q = q[(self.drone.identifier,self.drone.next_target())]
            

            #initially the packet remains with us
            max_action = None
            
            k = 1
            try:
                tot_n = n[(self.drone.identifier,self.drone.next_target())]
            except:
                tot_n = 0

            #loop for every neighbors
            for hello_packet, drone_istance in opt_neighbors:
                
                

                try:                
                
                    if (q[(drone_istance.identifier,hello_packet.next_target)] == max_q):
                        temp_dist = util.euclidean_distance(self.simulator.depot.coords, hello_packet.cur_pos)
                        if temp_dist < q_distance:
                            q_distance = temp_dist
                            max_action = drone_istance


                    #if we have a more reliable node
                    if (q[(drone_istance.identifier,hello_packet.next_target)] > max_q):
                        #select its best value for q function
                        max_q = q[(drone_istance.identifier,hello_packet.next_target)]
                        q_distance = util.euclidean_distance(self.simulator.depot.coords, hello_packet.cur_pos)
                        #select it
                        max_action = drone_istance
                        
                except Exception as e:
                    
                    q[(drone_istance.identifier,hello_packet.next_target)] = 0

                    if (q[(drone_istance.identifier,hello_packet.next_target)] == max_q):
                        temp_dist = util.euclidean_distance(self.simulator.depot.coords, hello_packet.cur_pos)
                        if temp_dist < q_distance:
                            q_distance = temp_dist
                            max_action = drone_istance

                    #if we have a more reliable node
                    if (q[(drone_istance.identifier,hello_packet.next_target)] > max_q):
                    
                        #select its best value for q function
                        max_q = q[(drone_istance.identifier,hello_packet.next_target)]
                        q_distance = util.euclidean_distance(self.simulator.depot.coords, hello_packet.cur_pos)

                        #select it
                        max_action = drone_istance



                k += 1
                try:
                    tot_n += n[(drone_istance.identifier,hello_packet.next_target)] 
                except Exception as e:
                    tot_n = tot_n

        self.drone.q = q
            
        #with epsilon probability we choose the random approach

                #with 1 - epsilon probability we choose the greedy approach
        import math
       # newEps =  min_epsilon + (math.tanh(n[self.drone.identifier, self.drone.next_target]/5) * (max_epsilon - min_epsilon)) 
    
        try:
         #    newEps = min_epsilon + ((1-math.tanh(tot_n/k))) * (max_epsilon - min_epsilon)
           # newEps =  min_epsilon + (math.exp(-1*(tot_n)/(k)) * (max_epsilon - min_epsilon)) 
         #   newEps = min_epsilon + 
            newEps = min_epsilon + ((1-((tot_n)/((tot_n)+k))) *(max_epsilon - min_epsilon))
       #     newEps = min_epsilon + (k /(k - tot_n)) *(max_epsilon - min_epsilon)
        except:
            newEps = min_epsilon

        rand2 = random.random()
        newEps = min_epsilon + (rand2 *(max_epsilon - min_epsilon))
              
#        newEps = min_epsilon
  #      print(newEps)
      #  newEps = max_epsilon
#        except:
       # newEps = min_epsilon + (0*(max_epsilon-min_epsilon))
        #newEps = max_epsilon
 #       newEps =  min_epsilon + (math.exp(-1*(tot_n**2)/(k**4)) * (max_epsilon - min_epsilon)) 

        if  rand< newEps:
            
            max_action = None
            
            #loop for every neighbors
            for hello_packet, drone_istance in opt_neighbors:
            
            #   exp_position = hello_packet.cur_pos  # without estimation, a simple geographic approach
            #   exp_position = self.compute_cross_point(hello_packet)

               #exp_position = self.compute_extimed_position(hello_packet)
               exp_distance = util.euclidean_distance(hello_packet.cur_pos, self.simulator.depot.coords)

             #  exp_distance = self.compute_distance_to_trajectory(hello_packet)
               

               
              # time_taken = exp_distance / hello_packet.speed
               
               
               
               #if (angle < 180 and angle_drone > 180):
                   
                #   continue
               

               if exp_distance < best_drone_distance_from_depot:
                   best_drone_distance_from_depot = exp_distance
                   max_action = drone_istance
                 #  time_taken_best = best_drone_distance_from_depot / hello_packet.speed
                   
             
        try:
           Reward = self.drone.Reward
        except:
           setattr(self.drone, "Reward", {})
           Reward = self.drone.Reward   
            
                    
        
        Reward[pkd.identifier] = max_action
        #return this random drone
        return max_action       
        
        #HERE BEGIN THE GEOGRAPHICAL ROUTING, BUT WE DON'T ARRIVE UNTIL HERE
        #TODO
        #A GOOD POSSIBLE IDEA IS TO COMBINE THIS REINFORCEMENT LEARNING WITH GEOGRAPHICAL ROUTING
        
        #we take all hello packets and all istances of drones
        for hello_packet, drone_istance in opt_neighbors:            
        
            
            import math
            
            
            
            
           
         #   exp_position = hello_packet.cur_pos  # without estimation, a simple geographic approach
         #   exp_position = self.compute_cross_point(hello_packet)

            exp_position = self.compute_extimed_position(hello_packet)
          #  exp_distance = util.euclidean_distance(exp_position, self.simulator.depot.coords)

            exp_distance = self.compute_distance_to_trajectory(hello_packet)
            

            
           # time_taken = exp_distance / hello_packet.speed
            
            
            
            #if (angle < 180 and angle_drone > 180):
                
             #   continue
            

            if exp_distance < best_drone_distance_from_depot:
                best_drone_distance_from_depot = exp_distance
                best_drone = drone_istance
              #  time_taken_best = best_drone_distance_from_depot / hello_packet.speed
                


     

        return best_drone

        # Only if you need --> several features:
        # cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
        #                                                width_area=self.simulator.env_width,
        #                                                x_pos=self.drone.coords[0],  # e.g. 1500
        #                                                y_pos=self.drone.coords[1])[0]  # e.g. 500
        # print(cell_index)
        action = None

        # self.drone.history_path (which waypoint I traversed. We assume the mission is repeated)
        # self.drone.residual_energy (that tells us when I'll come back to the depot).
        #  .....

        # Store your current action --- you can add several stuff if needed to take a reward later
        self.taken_actions[pkd.event_ref.identifier] = (action)

        return None  # here you should return a drone object!

    def print(self):
        """
            This method is called at the end of the simulation, can be usefull to print some
                metrics about the learning process
        """
        
        print(epsilon)
        print("Concluso")
        
        
        pass



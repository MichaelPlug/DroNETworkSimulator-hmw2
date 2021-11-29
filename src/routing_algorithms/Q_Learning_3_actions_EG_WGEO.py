"""
PER CHI LEGGERÃ€: NON SO COME IMPORTARE DA SRC.UTILITIES.CONFIG IL NUMERO 
DI DRONI CHE SONO INIZIALIZZATI LÃ€

L'IDEA ORA È CHE OGNI NODO È UNO STATO E LE AZIONI SONO TUTTI I POSSIBILI INVII AI
NODI VICINI PER UN DETERMINATO PACCHETTO

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


alpha = 0.5

gamma = 0.9

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
        
        print(self, drone, id_event, delay, outcome)
        
        
        try:
        	appo = self.drone.q
        except:
        	setattr(self.drone, 'q', {})
        		
        try:
         	appo = self.drone.s
        except: 
        	setattr(self.drone, 's', {})
        	
        try:
         	appo = self.drone.v_star
        except: 
        	setattr(self.drone, 'v_star', {})
	            
        #if the packet isn't still treated, then we train system for it
        if True:
   
            "Doubt: i don't know the utility of this"        
            if id_event in self.taken_actions:
                action = self.taken_actionself.drone.s[id_event]
                del self.taken_actionself.drone.s[id_event]
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
            #maybe also for all the path of packets to incentive themÃ¹            
            
            
            
            
            
            try:
                
                temp_, max_q = self.drone.s[(drone.identifier, id_event)]
            
            except Exception as e:
                
                temp_, max_q = 0, 0
            
            
            #in this way we also consider the action of return -1 
            if (temp_ == -1):
                
               
                try:
                
                    self.drone.q[(drone.identifier, 2)] = self.drone.q[(drone.identifier, 2)] + alpha*(R + gamma* max_q - self.drone.q[(drone.identifier, 2)] )
                
                except Exception as e:
                    
                    self.drone.q[(drone.identifier, 2)] = 10
                    
                    self.drone.q[(drone.identifier, 2)] = self.drone.q[(drone.identifier, 2)] + alpha*(R + gamma* max_q - self.drone.q[(drone.identifier, 2)] )
                
                
            #the packet remain to the node
            elif(temp_ == 0):
        
                
                try:
        
                    self.drone.q[(drone.identifier, 0)] = self.drone.q[(drone.identifier, 0)] + alpha*(R + gamma* max_q - self.drone.q[(drone.identifier, 0)])
                
                except Exception as e:
                    
                    self.drone.q[(drone.identifier, 0)] = 10
                    
                    self.drone.q[(drone.identifier, 0)] = self.drone.q[(drone.identifier, 0)] + alpha*(R + gamma* max_q - self.drone.q[(drone.identifier, 0)])
                
        
            
            #the packet was passed to another drone
            elif (temp_ == 1):
                
                
                try:
                
                    self.drone.q[(drone.identifier, 1)] = self.drone.q[(drone.identifier, 1)] + alpha*(R + gamma* max_q - self.drone.q[(drone.identifier, 1)] )
                
                except Exception as e:
                    
                    self.drone.q[(drone.identifier, 1)] = 10

                    self.drone.q[(drone.identifier, 1)] = self.drone.q[(drone.identifier, 1)] + alpha*(R + gamma* max_q - self.drone.q[(drone.identifier, 1)] )

                
            
            #self.drone.q[drone.identifier][self.drone.identifier] = self.drone.q[drone.identifier][self.drone.identifier] + R
        
            
            
    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """

        """ arg min score  -> geographical approach, take the drone closest to the depot """
        
        #we take our distance from the depot
        #best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
        #best_drone_distance_from_depot = self.compute_distance_to_trajectory_s()
        #initially drone closest is us (we take to the depot the
        #packet without any help)
        best_drone = None
        
        #we initialize an empty list, to find the maximum value
        l = []
        
        #we inizialize a temporanery variable to find maximum value from a list
        m = 0
        
        #we generate a random value, for the epsilon-greedy strategy
        rand = random.random()
        
        try:
        	appo = self.drone.q
        except:
        	setattr(self.drone, 'q', {})
        		
        try:
         	appo = self.drone.s
        except: 
        	setattr(self.drone, 's', {})
        	
        try:
         	appo = self.drone.v_star
        except: 
        	setattr(self.drone, 'v_star', {})
	            
                    
        
        #if we are in greedy case
        if (rand < 1 - epsilon):
        
            #we calculate what is the best action to perform, if it is 
            #the action of None, -1 or pass to any neighbour
            
            try:
            
                a = self.drone.q[(self.drone.identifier, 0)]
            
            except:
                
                self.drone.q[(self.drone.identifier, 0)] = 10
            
                
                a = self.drone.q[(self.drone.identifier, 0)]
            
            try:
            
                b = self.drone.q[(self.drone.identifier, 1)]
            
            except Exception as e:

                self.drone.q[(self.drone.identifier, 1)] = 10

                b = self.drone.q[(self.drone.identifier, 1)]

            try:

                c = self.drone.q[(self.drone.identifier, 2)]
            
            except Exception as e:
                
                self.drone.q[(self.drone.identifier, 2)] = 10
                
                c = self.drone.q[(self.drone.identifier, 2)]
            
            
            
            #if the best action is to maintain the packet and remain to
            #our trajectory
            if (a >= b and a >= c):
                
                #we calculate the maximum value of the three possible actions
                #NOT NECESSARY TRY-EXCEPT, EXECUTED JUST BEFORE
                l = [self.drone.q[(self.drone.identifier, 0)], self.drone.q[(self.drone.identifier, 1)], self.drone.q[(self.drone.identifier, 2)]]
                m = max(l)
                
                #we set, in a global variable, that this drone 
                #for this packet has perform the action to maintain
                #the packet and to remain to its trajectory and it is
                #saved also the maximum possible value
                self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (0, m)
                
                
                
                try:
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m
                
                except Exception as e:
                    
                    self.drone.v_star[self.drone.identifier] = 0
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m
                
                
                #do anything
                return None
            
            #if the better action is to go to the depot, then this means
            #that we want do the same of the previous if, in practise
            if (c >= a and c >= b):
                
                #we take the maximum value, for the reward calculation
                #NOT NECESSARY TRY-EXCEPT, EXECUTED JUST BEFORE
                l = [self.drone.q[(self.drone.identifier, 0)], self.drone.q[(self.drone.identifier, 1)], self.drone.q[(self.drone.identifier, 2)]]
                m = max(l)
                
                #we save this result, in practise
                self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (-1, m)
                
                try:
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m
                
                except Exception as e:
                    
                    self.drone.v_star[self.drone.identifier] = 0
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m
                
                
                #at the end we perform the action to go to the depot, so
                #we left the mission for this purpose
                return -1
            
            #if the best choice to do is to pass the packet to the neighbors
            if (b >= a and b >= c):
                
                "FIRST PHASE -- TAKE MAX Rs FOR a -- SLIDE 32"
                #we take the maximum value, for the reward calculation
                #NOT NECESSARY TRY-EXCEPT, EXECUTED JUST BEFORE
                l = [self.drone.q[(self.drone.identifier, 0)], self.drone.q[(self.drone.identifier, 1)], self.drone.q[(self.drone.identifier, 2)]]
                m = max(l)
                
                #we initialize two differente variables to do some things
                sum_v_star = 0
                max_v_star = 0
                
                
                "SECOND PHASE PHASE -- SUM OF PROBABILITIES AND V* -- SLIDE 32"
                #we meed to know sum of v* of all neighbors
                for hello_packet, drone_istance in opt_neighbors:
                    
                    sum_v_star = sum_v_star + self.drone.v_star[drone_istance.identifier]
                
                
                try:
                    
                    #set the v_star attribute
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m + gamma*sum_v_star
                
                except Exception as e:
                    
                    self.drone.v_star[self.drone.identifier] = 0
                    
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m + gamma*sum_v_star
                
                
                max_v_star = self.drone.v_star[self.drone.identifier]
                
                max_action = None

                "THIRD PHASE -- IDENTIFY THE MAX FOR a OF EVERY Q(S',a) (all neighbors)"
                "SLIDE 42"                
                #loop for every neighbors
                for hello_packet, drone_istance in opt_neighbors:
                    

                    #because we must identify max_a Q(S' , a)
                    if (self.drone.v_star[drone_istance.identifier] > max_v_star):
                        
                        max_v_star = self.drone.v_star[drone_istance.identifier]     
                        max_action = drone_istance
                        
                #max of possible actions
                return_m = m
                
                "FOURTH PHASE -- SELECT THE BEST NEIGHBOR POSSIBLE, WITH"
                "HIGHEST VALUE LEARNED"
                for i in range(3):
                    
                    for hello_packet, drone_istance in opt_neighbors:
                     
                        try:
                            
                            
                            if (self.drone.q[(drone_istance.identifier, i)] > return_m):
                             
                             
                                return_m = self.drone.q[(drone_istance.identifier, i)]
                        
                        
                        except Exception as e:
                            
                            
                            self.drone.q[(drone_istance.identifier, i)] = 10
                        
                        
                            if (self.drone.q[(drone_istance.identifier, i)] > return_m):
                             
                                return_m = self.drone.q[(drone_istance.identifier, i)]
                   
                        
                #save everything for the capturing of the reward in 
                #successive phase of feedback
                if (max_action == None):
                    
                    self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (0, return_m)
                
                else:
                    
                    self.drone.s[(max_action.identifier, pkd.event_ref.identifier)] = (1, return_m)
                
                
                return max_action
                
                
                
                
            
            
        #in the random case (epsilon case)    
        else:
            
            
            
            
            """ arg min score  -> geographical approach, take the drone closest to the depot """
            best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
            max_action = None

            
            l = []
            
            
            #we take the maximum value, for the reward calculation
            try:
                
                l.append(self.drone.q[(self.drone.identifier, 0)])
                
            except Exception as e:
                
                self.drone.q[(self.drone.identifier, 0)] = 10
                
                l.append(self.drone.q[(self.drone.identifier, 0)])
                
            
            try:
                
                l.append(self.drone.q[(self.drone.identifier, 1)])
                
            except Exception as e:
                
                self.drone.q[(self.drone.identifier, 1)] = 10
                
                l.append(self.drone.q[(self.drone.identifier, 1)])
                
                
            try:
                
                l.append(self.drone.q[(self.drone.identifier, 2)])
                
            except Exception as e:
                
                self.drone.q[(self.drone.identifier, 2)] = 10
                
                l.append(self.drone.q[(self.drone.identifier, 2)])
                
            
    

            # Action MOVE is identified by -1
            if len(opt_neighbors) == 0:
                
                
                m = max(l)
                
                #we save this result, in practise
                self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (-1, m)
                
                try:
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m
                
                except Exception as e:
                    
                    self.drone.v_star[self.drone.identifier] = 0
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m
                
                
                #at the end we perform the action to go to the depot, so
                #we left the mission for this purpose
                return -1

            
            "FIRST PHASE -- TAKE MAX Rs FOR a -- SLIDE 32"
            #we take the maximum value, for the reward calculation
            #this is the ELSE part
            
            m = max(l)
            
            #we initialize two differente variables to do some things
            sum_v_star = 0
            max_v_star = 0
            
            
            "SECOND PHASE PHASE -- SUM OF PROBABILITIES AND V* -- SLIDE 32"
            #we meed to know sum of v* of all neighbors
            for hello_packet, drone_istance in opt_neighbors:
                
                
                try:
                    
                
                    sum_v_star = sum_v_star + self.drone.v_star[drone_istance.identifier]
            
                except Exception as e:
                    
                    self.drone.v_star[drone_istance.identifier] = 0
                    
                    sum_v_star = sum_v_star + self.drone.v_star[drone_istance.identifier]
            
            
            
            try:
                
                #set the v_star attribute
                self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m + gamma*sum_v_star
            
            except Exception as e:
                
                self.drone.v_star[self.drone.identifier] = 0
                
                self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m + gamma*sum_v_star
            
            
            max_v_star = self.drone.v_star[self.drone.identifier]
            
            max_action = None

            #max of possible actions
            return_m = m
            
            
            
            


            for hpk, drone_istance in opt_neighbors:
                
                exp_position = hpk.cur_pos  # without estimation, a simple geographic approach
                exp_distance = util.euclidean_distance(exp_position, self.simulator.depot.coords)
                if exp_distance < best_drone_distance_from_depot:
                    best_drone_distance_from_depot = exp_distance
                    max_action = drone_istance
                    
                    try:
                    
                        return_m = self.drone.q[(drone_istance.identifier, 1)]
                    
                    except Exception as e:
                        
                        self.drone.q[(drone_istance.identifier, 1)] = 10
                        
                        return_m = self.drone.q[(drone_istance.identifier, 1)]
                    
                    


            #save everything for the capturing of the reward in 
            #successive phase of feedback
            if (max_action == None):
                
                
                m = max(l)
                
                #we set, in a global variable, that this drone 
                #for this packet has perform the action to maintain
                #the packet and to remain to its trajectory and it is
                #saved also the maximum possible value
                self.drone.s[(self.drone.identifier, pkd.event_ref.identifier)] = (0, return_m)
                
                
                
                try:
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m
                
                except Exception as e:
                    
                    self.drone.v_star[self.drone.identifier] = 0
                    self.drone.v_star[self.drone.identifier] = self.drone.v_star[self.drone.identifier] + m
                
                
                
                
            
            else:
                
                self.drone.s[(max_action.identifier, pkd.event_ref.identifier)] = (1, return_m)
            
            

            return max_action
    
    
        """
        
        FINE
        
        """
        
       
     
        

    def print(self):
        """
            This method is called at the end of the simulation, can be usefull to print some
                metrics about the learning process
        """
        
        """
        print("Hello", q)
        print("Alo", n)
        print("Salut", c)
        print(epsilon)
        """
        
        pass

    
    
    def compute_extimed_position(self, hello_packet):
        """ estimate the current position of the drone """

        # get known info about the neighbor drone
        hello_message_time = hello_packet.time_step_creation
        known_position = hello_packet.cur_pos
        known_speed = hello_packet.speed
        known_next_target = hello_packet.next_target

        # compute the time elapsed since the message sent and now
        # elapsed_time in seconds = elapsed_time in steps * step_duration_in_seconds
        elapsed_time = (self.simulator.cur_step - hello_message_time) * self.simulator.time_step_duration  # seconds

        # distance traveled by drone
        distance_traveled = elapsed_time * known_speed

        # direction vector
        a, b = np.asarray(known_position), np.asarray(known_next_target)
        if np.linalg.norm(b - a) != 0:
        	v_ = (b - a) / np.linalg.norm(b - a)
        else:
        	v_ = 0

        # compute the expect position
        c = a + (distance_traveled * v_)

        return tuple(c)

    def compute_distance_to_trajectory_s(self):
        p1 = np.array([self.drone.coordself.drone.s[0], self.drone.coordself.drone.s[1]])
        p2 = np.array([self.drone.next_target()[0], self.drone.next_target()[1]])
        p3 = np.array([self.drone.depot.coordself.drone.s[0],self.drone.depot.coordself.drone.s[1]])

        
        if np.linalg.norm(p2-p1) != 0:
        	return np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
        else: 
        	return 0

    def compute_distance_to_trajectory(self, hello_packet):

        exp_position = self.compute_extimed_position(hello_packet)
       # exp_position = hello_packet.cur_pos

        #MAYBE IT SHOULD BE p1 = np.array([exp_position[0][0], exp_position[0][1]])
        p1 = np.array([exp_position[0], exp_position[1]])
        p2 = np.array([hello_packet.next_target[0], hello_packet.next_target[1]])
        p3 = np.array([self.drone.depot.coordself.drone.s[0],self.drone.depot.coordself.drone.s[1]])
        
        if np.linalg.norm(p2-p1) != 0:
        	return np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
        else: 
        	return 0

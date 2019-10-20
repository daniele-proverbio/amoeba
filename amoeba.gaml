/**
* Name: amoeba
* Author: daniele proverbio, luca gallo, barbara passalacqua
* Description: Behaviour of social Amoeba Dictyostelium Discoideum
* Tags: biology, robots, clustering
*/

model amoeba

global torus: torus_environment
{
	//parameters
    int number_of_amoebas <- 2640 min: 1 max: 2000000 ;   		//number of amoeba menagement   (2640 standard)
    int number_of_obstacles <- 0 min: 0 max: 20 ;        		//obstacles
    int number_of_bacteria <- 2 min: 1 max: 10 ; 	    		//bacteria -> food
    float amoeba_maximal_speed <- 1.4 min: 0.001 max: 2.0 ;     //speed of amoeba in micron/min  (1.4 x 2640)
    float cAMP_maximal_speed <- 4.7 ;     						//speed of amoeba in micron/min   (4.7 x 2640)
    int max_time <- 10 min: 1 max: 15 ;							//time after one shot of signal
    int dimension <- 469 ;										//of the environment (standard 469)
    float bacteriaGrowthProb <- 0.0125 min: 0.0 max: 1.0 ; 		//rate of growth for bacteria
    float agitationProb <- 0.001 min: 0.0 max: 1.0 ;			//probability of not following the signal
    float amoeba_dimension <- 0.9 min: 0.01 max: 3.0 ;
    //-----------------------------------------
    // rules
    bool torus_environment <- false;
    bool apply_avoid <- true;

    //enviroment shape
    geometry shape <- square(dimension);

    //-----------------------------------------
    //colors
    rgb black const:true <- rgb('black');
    rgb white const:true <- rgb('white');
    rgb green const:true <- rgb('green');
    rgb blue const:true <- rgb('blue');
    rgb light_sierra const:true <- rgb('#F87431');
    rgb dark_orange const:true <- rgb('#F88017');
    rgb sienna const:true <- rgb('#C35817');
    rgb chocolate const:true <- rgb('#7E2217');

    //----------------------------------
    //experimentalist's measurements:

    //statistical variables
    //float box <- 1.0 update: ((x_max-x_min)*(y_max-y_min))/(dimension^2);  //box clustering
    float var_x -> {variance(amoeba collect each.location.x)/(dimension^2)}; //variance on x axis
    float var_y -> {variance(amoeba collect each.location.y)/(dimension^2)}; //variance on y axis




    //-----------------------------------
    //other interesting variables    
    //list<int> number_cAMP-> {amoeba collect each.quanti};
    list<float> ics -> {amoeba collect each.location.x};
    list<float> ipsilon -> {amoeba collect each.location.y};


    //-----------------------------------
    //set agents and enviroment
    init
    {
        create amoeba number: number_of_amoebas   //amoeba setting
        {
             location <- {rnd (dimension - 2) + 1, rnd (dimension -2) + 1 };
             signalTime <- max_time;
        }



        create obstacle number:number_of_obstacles
        {
			shape <- rectangle(1+rnd(15), 1+rnd(15));
		}

        ask bacteria_cell       //bacteria setting
        {
            maxBacteria <- number_of_bacteria; //control of switch for colors
            bacteria <- 2; //rnd(maxBacteria);
            color <- [white,light_sierra,dark_orange,sienna,chocolate] at bacteria;
        }
    }

	reflex save_data when: every(1 #cycle){
		//save the following variables into the given csv file. Note that each time the save statement is used, a new line is added at the end of the file.
			save (ics) to: "../results/amoeba_x_600_100_1.csv" type:"csv" rewrite: false;
			save (ipsilon) to: "../results/amoeba_y_600_100_1.csv" type:"csv" rewrite: false;

	}

    reflex stop_simulation when: (time > 4000){
		do pause;
	}
}


    //set grid of bacteria (acting as food/noise for the amoebas)
    grid bacteria_cell width: dimension height: dimension neighbors: 8 use_individual_shapes: false use_regular_agents: false
    {
        int maxBacteria;
        int bacteria update:(flip(bacteriaGrowthProb))? bacteria + 1 : bacteria max: maxBacteria;
        rgb color update: [white,light_sierra,dark_orange,sienna,chocolate] at bacteria;
    }

    //----------------------------
    //specie Amoeba definition
    species amoeba skills: [moving] control: fsm
    {
        int signalTime max: max_time;
        bacteria_cell place update: bacteria_cell(location);
        float range <- amoeba_dimension;   								 //practical dimension of an amoeba
        float range_obs <- 0.1;
        bool thereIsFood;
        float speed <- amoeba_maximal_speed;
        int quanti<-0;						 //amoeba speed

        reflex tic   //update internal time
        {
            signalTime <- signalTime + 1;
        }

        reflex isThereFood  //check whether there is food -> reflex = each cycle
        {
        	if(place.bacteria != 0)
        	{
        		do eat;
        		thereIsFood <- true;
        	}
        	else
        	{
        		thereIsFood <- false;
        	}
        }

        reflex avoid when: apply_avoid  //avoid obstacles by ''bouncing''
        {
			list<obstacle> nearby_obstacles <- (obstacle overlapping (circle (range_obs)) );
			if(length(nearby_obstacles)!=0)
			{
				self.heading <- abs(180-self.heading);
			}
		}

        action eat
        {
        	place.bacteria <- place.bacteria -1;
        }

        action shootSignal
        {
        	int angle <- 0;
            create cAMP number: 8
            {
            	location <- myself.location;
                self.heading <- angle + rnd(45) - 22.0;  //smearing of shooting
                angle <- angle + 45;
            }
            signalTime <- 0;
        }

        action listenToSignal
        {
            list<cAMP> nearby_cAMP <- (cAMP overlapping(circle(range)));
            quanti <- length(nearby_cAMP);
            if(length(nearby_cAMP)!=0)
            {
            		cAMP nearest <- nearby_cAMP[0];
            		self.heading <- (180 + nearest.heading);  //adjust direction
            		ask nearby_cAMP{do die;}				  //adsorb cAMP molecule
            }

        }

        action followSignal
        {
           do wander amplitude: 20.0; 	//occasional deviations from deterministic path
        }


        //states of amoeba
        state wandering initial: true     //default
        {
            do wander;
            transition to: starving when: !thereIsFood;
        }

        state starving					  //no food -> starving; actions of shooting and listening to signals
        {
            if(signalTime=max_time)
            {
                do shootSignal;
            }
            else if(flip(1-agitationProb))
            {
            	do listenToSignal;
                do followSignal;
            }
            else
            {
                do wander;
            }
            transition to: wandering when: thereIsFood;
        }

        state surrounded       //less DF -> apply cohesion
        {

        }

        aspect default         //aspect
        {
        	draw circle(0.5) color: green;
        }
    } //end amoeba


    //-------------------------------------------
    //specie cAMP molecule (signal) definition
    species cAMP skills: [moving]
    {
        float speed <- cAMP_maximal_speed;
        float range <- 0.1;

        reflex checkCell
        {
            list<obstacle> nearby_obstacles <- (obstacle overlapping (circle (range)) );
            if(length(nearby_obstacles)!=0)
            {
                do die;
            }
        }

        reflex dieOnBorder   //being absorbed by borders
        {
        	if(location.x <= 1 or location.x >= (dimension-1) or location.y <= 1 or location.y >= (dimension-1))
        		{
        			do die;
        		}
        }

        reflex moveStraight  //and nothing else
        {
            do move;
        }

        aspect default
        {
        	draw circle(0.1) color: blue;
        }
    }


    //-----------------------
    //specie obstacle
    species obstacle
    {
        aspect default
        {
			draw shape color: #gray border: #black;
		}
    }


 //---------------------------------------
 //definition of the experiment
experiment Aggregation  type: gui
{
	parameter 'Number of amoebas' var: number_of_amoebas;
	parameter 'Number of obstacles' var: number_of_obstacles;
	parameter 'Speed of amoebas' var: amoeba_maximal_speed;
	parameter 'Agitation Probability' var: agitationProb;
	parameter 'Number of Bacteria' var: number_of_bacteria;
	parameter 'Bacteria Growth Probability' var: bacteriaGrowthProb;
	parameter 'Width/Height of the Environment' var: dimension ;
	parameter 'Toroidal Environment ?'  var: torus_environment ;
	parameter 'Apply Avoidance ?' var: apply_avoid ;

	// multiple simulations
        //float seedValue<-0.1*rnd(1000);
        //float seed<-seedValue;
        //init {
        //       create simulation with:[seed::rnd(1000)];
        //}

    output
    {
        display grille
        {
            grid bacteria_cell;
            species amoeba;
            //species obstacle;
            //species cAMP;
        }
        //display Chart_Box_Clustering
        //{
        //        chart name: "Box Clustering" type: series background: rgb("white") axes: rgb("black") position: {0,0} size: {1.0,1.0}
        //        {
        //           data "Box Clustering" color: rgb("blue") value: 1-box style: spline ;
        //        }
        //}

        display Chart1_Variance
        {
        		chart name: "Variance" type: series background: rgb("white") axes: rgb("black") position: {0,0} size: {1.0,1.0}
                {
                    data "Variance_x" color: rgb("red") value: var_x style: spline ;
                    data "Variance_y" color: rgb("green") value: var_y style: spline ;
                }

         }

    }
}

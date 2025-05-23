package fr.uga.pddl4j.montecarlo;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerConfiguration;
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
//import fr.uga.pddl4j.planners.SearchStrategy;
//import fr.uga.pddl4j.planners.statespace.search.StateSpaceSearch;
import fr.uga.pddl4j.planners.statespace.HSP;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
//import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
//import org.apache.logging.log4j.core.tools.picocli.CommandLine.Command;

import picocli.CommandLine;

import java.io.*;
import java.util.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.stream.Stream;


/**
 * The following class implements a Monte Carlo Tree Search Planner. It makes use of PDDL4J 
 *
 * @author Takudzwa Togarepi
 * @version 1.0 - 21.05.2025
 */
@CommandLine.Command(name = "MonteCarloTreeSearch",
    version = "MonteCarlo 1.0",
    description = "Solves a specified planning problem using MCTS with pure random walk planning procedure",
    sortOptions = false,
    mixinStandardHelpOptions = true,
    headerHeading = "Usage:%n",
    synopsisHeading = "%n",
    descriptionHeading = "%nDescription:%n%n",
    parameterListHeading = "%nParameters:%n",
    optionListHeading = "%nOptions:%n")
public class MonteCarloTreeSearch extends AbstractPlanner {

    /**
     * The class logger.
     */
    private static final Logger LOGGER = LogManager.getLogger(MonteCarloTreeSearch.class.getName());

    // We need the following for our planner config.
    private static final String HEURISTIC_STRING = "HEURISTIC";

    /** As a default heuristic we use FF heuristic. 
     * 
    */
     
    public static final StateHeuristic.Name DEFAULT_HEURISTIC_NAME = StateHeuristic.Name.FAST_FORWARD;

    /*
     * This is another property we will use for planner config.
     */
    public static final String EXPLORATION_CONSTANT_SETTING = "EXPLORATION_CONSTANT";

    /* We set the default value of the exploration heuristic to 1 */
    
    public static final double DEFAULT_EXPLORATION_CONSTANT_VALUE = 1.0;

    /* Number of Walks */

    public static final String NUMBER_OF_WALKS_SETTING = "NUMBER_OF_WALKS";

    /*
     * We set the default number of walks to 50.
     */
    
     public static final int DEFAULT_NUMBER_OF_WALKS = 50;

    /*
     * We will need the length of walks in the config.
     */
    public static final String LENGTH_OF_EACH_WALK_SETTING = "LENGTH_OF_WALK";

    /*
     * We set the default value of the length of walks to 10.
     */
    public static final int DEFAULT_LENGTH_OF_EACH_WALK = 10;

    public static final String ALPHA_SETTING = "ALPHA";

    /* higher alpha gives more emphasis on recent progress. */
    public static final double DEFAULT_ALPHA  = 0.9;

    public static final String MAX_STEPS = "MAX_STEPS";

    /** restarts after 7 steps if no improvement is observed. */
    private static final int DEFAULT_MAX_STEPS = 7;

    /** c is the exploration constant for UCT.*/
    private double c;

    private StateHeuristic.Name heuristic;

    /** getC returns the value of c, the exploration constant
     * @return the value of c, the exploration constant
     */
    public final double getC(){
        return this.c;
    }

    private int lengthOfWalk;

    /** returns the length of each walk
     * @return the length of each walk.
    */
    public final int getLengthOfEachWalk(){
        return this.lengthOfWalk;
    }

    private int numberOfWalks;

    /** returns number of walks.
     * @return number of walks.
    */
    public final int getNumberOfWalks(){
        return this.numberOfWalks;
    }

    /**
     * returns the heuristic name used by the planner.
     *
     * @return tthe heuristic name used by the planner.
     */
    public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }
    
    /** 
     * This creates a planner using the default config. 
     */
    public MonteCarloTreeSearch(){
        this(MonteCarloTreeSearch.getDefaultConfiguration());
    }

    /** 
     * This creates a planner using a specific config.
     * @param configuration the config of the planner
     */

    public MonteCarloTreeSearch(final PlannerConfiguration configuration){
        super();
        this.setConfiguration(configuration);
    }

    /** This option sets the exploration constant from the command line.
     * @param c should be greater than 0 that is c > 0.
     * @throws IllegalArgumentException if c is less than 0.
     */

    @CommandLine.Option(names = {"-c", "--exploration_constant"}, defaultValue = "1.0",paramLabel = "<exploration_constant>", description = "The exploration constant is set to 1.0 by default.")
    public void setExplorationConstant(final double exploration_constant){
        if(exploration_constant <= 0){
            throw new IllegalArgumentException("exploration constant is less than or equal to 0.");
        }
        this.c = exploration_constant;
    }

    /**
     * This option is for setting the heuristic name from the command line. The default option is Fast Forward.
     * @param heuristic_name the name of the heuristic used by the planner.
     */
    @CommandLine.Option(names = {"-e", "--heuristic"}, defaultValue = "FAST_FORWARD", description = "We should set the heuristic to be used by the planner.")
    public void setHeuristic(StateHeuristic.Name heuristic){
        this.heuristic = heuristic;
    }

    /**
     * This option sets the number of walk and the number of walks must be greater than 0.
     * @param numberOfWalks the number of walks.
     * @throws IllegalArgumentException if number of walks are less than 0.
     */
    @CommandLine.Option(names = {"-nw", "numberOfWalks"}, defaultValue = "50", paramLabel = "<numberOfWalks>", description = "The number of random walks is by default set to 50." )
    public void setNumberOfWalks(final int numberOfWalks){
        if(numberOfWalks <= 0){
            throw new IllegalArgumentException("Your number of walks is less than 0. It should be strictly a positive value.");
        }
        this.numberOfWalks = numberOfWalks;
    }

    /**
     * This option sets the length of each walk and the length of each walk must be greater than 0.
     * @param lengthOfWalk the length of each walk.
     * @throws IllegalArgumentException if the length of each walk is less than 0.
     */
    @CommandLine.Option(names = {"-lw", "lengthOfWalk"}, defaultValue = "10", paramLabel = "<lengthOfWalk>", description = "The length of each random walk is by default set to 10." )
    public void setLengthOfWalk(final int lengthOfWalk){
        if(lengthOfWalk <= 0){
            throw new IllegalArgumentException("Your number of walks is less than 0. It should be strictly a positive value.");
        }
        this.lengthOfWalk = lengthOfWalk;
    }

    /** 
     * returns planner default arguments.
     * @return planner default arguments.
     */
    public static PlannerConfiguration getDefaultConfiguration(){
        PlannerConfiguration configs = Planner.getDefaultConfiguration();
        configs.setProperty(MonteCarloTreeSearch.HEURISTIC_STRING, MonteCarloTreeSearch.DEFAULT_HEURISTIC_NAME.toString());
        configs.setProperty(MonteCarloTreeSearch.EXPLORATION_CONSTANT_SETTING, Double.toString(MonteCarloTreeSearch.DEFAULT_EXPLORATION_CONSTANT_VALUE));
        configs.setProperty(MonteCarloTreeSearch.LENGTH_OF_EACH_WALK_SETTING, Integer.toString(MonteCarloTreeSearch.DEFAULT_LENGTH_OF_EACH_WALK));
        configs.setProperty(MonteCarloTreeSearch.NUMBER_OF_WALKS_SETTING, Integer.toString(MonteCarloTreeSearch.DEFAULT_NUMBER_OF_WALKS));
        return configs;
    }

    /** 
     * returns planner arguments of an instance.
     * @return planner arguments of an instance.
     */
    @Override
    public PlannerConfiguration getConfiguration(){
        PlannerConfiguration configs = Planner.getDefaultConfiguration();
        configs.setProperty(MonteCarloTreeSearch.HEURISTIC_STRING, MonteCarloTreeSearch.DEFAULT_HEURISTIC_NAME.toString());
        configs.setProperty(MonteCarloTreeSearch.EXPLORATION_CONSTANT_SETTING, Double.toString(MonteCarloTreeSearch.DEFAULT_EXPLORATION_CONSTANT_VALUE));
        configs.setProperty(MonteCarloTreeSearch.LENGTH_OF_EACH_WALK_SETTING, Integer.toString(MonteCarloTreeSearch.DEFAULT_LENGTH_OF_EACH_WALK));
        configs.setProperty(MonteCarloTreeSearch.NUMBER_OF_WALKS_SETTING, Integer.toString(MonteCarloTreeSearch.DEFAULT_NUMBER_OF_WALKS));
        return configs;
    }

    /**
     * This sets the arguments of the planner and if one is not specified the default will be used.
     * @param planner_arguments the arguments that need to be set for the planner.
     */
    @Override
    public void setConfiguration(final PlannerConfiguration configuration){
        super.setConfiguration(configuration);
        if(configuration.getProperty(MonteCarloTreeSearch.EXPLORATION_CONSTANT_SETTING) == null){
            this.setExplorationConstant(MonteCarloTreeSearch.DEFAULT_EXPLORATION_CONSTANT_VALUE);
        }
        else{
            this.setExplorationConstant(Double.parseDouble(configuration.getProperty(MonteCarloTreeSearch.EXPLORATION_CONSTANT_SETTING)));
        }

        if(configuration.getProperty(MonteCarloTreeSearch.HEURISTIC_STRING) == null){
            this.setHeuristic(MonteCarloTreeSearch.DEFAULT_HEURISTIC_NAME);
        }
        else{
            this.setHeuristic(StateHeuristic.Name.valueOf(configuration.getProperty(MonteCarloTreeSearch.HEURISTIC_STRING)));
        }

        if(configuration.getProperty(MonteCarloTreeSearch.NUMBER_OF_WALKS_SETTING) == null){
            this.setNumberOfWalks(MonteCarloTreeSearch.DEFAULT_NUMBER_OF_WALKS);
        }
        else{
            this.setNumberOfWalks(Integer.parseInt(configuration.getProperty(MonteCarloTreeSearch.NUMBER_OF_WALKS_SETTING)));
        }

        if(configuration.getProperty(MonteCarloTreeSearch.LENGTH_OF_EACH_WALK_SETTING) == null){
            this.setLengthOfWalk(MonteCarloTreeSearch.DEFAULT_LENGTH_OF_EACH_WALK);
        }
        else{
            this.setLengthOfWalk(Integer.parseInt(configuration.getProperty(MonteCarloTreeSearch.LENGTH_OF_EACH_WALK_SETTING)));
        }
        
    }

    /**
     * The following checks if the planner has valid arguments
     * @return <code>true</code> if all arguments are valid else it returns <code>false</code>
     */
    public boolean hasValidConfiguration(){
        return super.hasValidConfiguration() && this.getC() > 0.0 && this.getHeuristic() != null && this.getNumberOfWalks() > 0 && this.getLengthOfEachWalk() > 0;

    }

    @Override
    public Problem instantiate(DefaultParsedProblem problem){
        final Problem prob = new DefaultProblem(problem);
        prob.instantiate();
        return prob;
    }
    
    /**
     * We solve the problem with MCTS which uses pure random walks as well.
     * If we find a solution we return the plan.
     * 
     * @throws ProblemNotSupportedException when we have a problem which the planner does not support.
     */
    @Override
    public Plan solve(final Problem problem) throws ProblemNotSupportedException{
        LOGGER.info("Starting MCTS \n");
        final long begin = System.currentTimeMillis();
        final Plan plan = this.montecarlo(problem);
        final long end = System.currentTimeMillis();

        if(plan != null){
            LOGGER.info("Successfully completed MCTS!" + plan.size() +" \n" );
            this.getStatistics().setTimeToSearch(end - begin);
        }
        else{
            LOGGER.info("The MCTS failed\n");
        }
        return plan;
    }

    public Plan montecarlo(Problem problem) throws ProblemNotSupportedException{
        
        if(!this.isSupported(problem)){
            throw new ProblemNotSupportedException("Problem not supported!");
        }
        final StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);

        State init = new State(problem.getInitialState());
        Node n = new Node(init, null, -1, 0, 0, heuristic.estimate(init, problem.getGoal()));

        double hMin = n.getHeuristic();
        int counter = 0;
        while(!n.satisfy(problem.getGoal())){
            if(counter >= DEFAULT_MAX_STEPS || getActions(problem,n).isEmpty()){
                n = new Node(init, null, -1, 0, 0, heuristic.estimate(init, problem.getGoal()));
            counter = 0;
            }
            n = pureRandomWalkAlgo(problem, n, heuristic);
            if(n.getHeuristic()< hMin){
                hMin = n.getHeuristic();
                counter = 0;
            }
            else{
                counter++;
            }
        }
        return extractPlan(n, problem);

    }
    
    /**
     * Returns if a specified problem is supported by the planner. Just ADL problem can be solved by this planner.
     *
     * @param problem the problem to test.
     * @return <code>true</code> if the problem is supported <code>false</code> otherwise.
     */
    @Override
    public boolean isSupported(Problem problem){
        return(problem.getRequirements().contains(RequireKey.ACTION_COSTS)
        || problem.getRequirements().contains(RequireKey.CONSTRAINTS)
        || problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
        || problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
        || problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
        || problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
        || problem.getRequirements().contains(RequireKey.FLUENTS)
        || problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
        || problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
        || problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
        || problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
        || problem.getRequirements().contains(RequireKey.PREFERENCES)
        || problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
        || problem.getRequirements().contains(RequireKey.HIERARCHY))
        ? false : true;
    }

     /**
     * Extracts a search from a specified node.
     *
     * @param node    the node.
     * @param problem the problem.
     * @return the search extracted from the specified node.
     */
    private Plan extractPlan(final Node node, final Problem problem){
        Node n = node;
        final Plan plan = new SequentialPlan();
        while(n.getAction() != -1){
            final Action a = problem.getActions().get(n.getAction());
            plan.add(0, a);
            n = n.getParent();
        }
        return plan;
    }
    /**
     * Implements the pure random walk algorithm, Algorithm 2 in @see <a href="http://pddl4j.imag.fr/repository/exercices/resources/arvand.pdf"> http://pddl4j.imag.fr/repository/exercices/resources/arvand.pdf </a>
     * @param p the planning problem.
     * @param s the node.
     * @param heuristic the heuristic.
     * @return A new node.
     */
    public Node pureRandomWalkAlgo(Problem p, Node s, StateHeuristic heuristic){
        double hMin = Double.MAX_VALUE;
        Node sMin = null;
        for(int i = 0; i < numberOfWalks; i++){
            Node sPrime = s;
            for(int j = 1; j < lengthOfWalk; j++){
                List<Action> A = this.getActions(p,sPrime);
                if(A.isEmpty())
                 break;
                Action a = selectRandomAction(A);
                sPrime = useAction(p, sPrime, a, heuristic);
                if(sPrime.satisfy(p.getGoal()))
                 return sPrime;

            }
            if(sPrime.getHeuristic() < hMin){
                hMin =sPrime.getHeuristic();
                sMin = sPrime;
            }
        }
        return sMin == null ? s : sMin;
    }

    private List<Action> getActions(Problem p,Node n){
        List<Action> actions = p.getActions();
        List<Action> applicableActions = new ArrayList<>();
        for(Action a : actions){
            if(a.isApplicable(n)){
                applicableActions.add(a);
            }
        }
        return applicableActions;
    }

    /**
     * 
     * @param listActions a list of actions.
     * @return returns a random action
     */
    private Action selectRandomAction(List<Action> listActions){
        Collections.shuffle(listActions);
        return listActions.get(0);
    }

    public Node useAction(Problem p, Node n, Action a, StateHeuristic heuristic){
        State s = new State(n);
        s.apply(a.getConditionalEffects());
        Node child = new Node(s, n, p.getActions().indexOf(a), n.getCost() + 1, n.getDepth() + 1, 0);
        child.setHeuristic(heuristic.estimate(child, p.getGoal()));
        
        return child;
    }

    /**
     * Compares performance of HSP and MCTS
     * @param args command line arguments
     */
    private static void comparePerformance() throws IOException{
        LOGGER.info("Starting performance comparison of HSP and MCTS");
        MonteCarloTreeSearch planner = new MonteCarloTreeSearch();
        HSP hspPlanner = new HSP();
        StringBuilder csvFile = new StringBuilder("Domain, problem, MCTS_time(ms),MCTS_plan_length, HSP_time(ms), HSP_plan_length \n");

        Map<String, String[]> domains = Map.of(
            "src/test/resources/benchmarks/pddl/ipc2000/blocks/strips-typed/domain.pddl", new String[]{"blocks","src/test/resources/benchmarks/pddl/ipc2000/blocks/strips-typed"},
            "src/test/resources/benchmarks/pddl/ipc2000/logistics/strips-typed/domain.pddl", new String[]{"logistics", "src/test/resources/benchmarks/pddl/ipc2000/logistics/strips-typed"},
            //"src/test/resources/benchmarks/pddl/ipc2002/depots/strips-automatic/domain.pddl", new String[]{"depots","src/test/resources/benchmarks/pddl/ipc2002/depots/strips-automatic"},
            "src/test/resources/benchmarks/pddl/ipc1998/gripper/strips/domain.pddl", new String[]{"gripper", "src/test/resources/benchmarks/pddl/ipc1998/gripper/strips"}

        );

        for(Map.Entry<String, String[]> domain : domains.entrySet()){
            String domainPath = domain.getKey();
            String domainName = domain.getValue()[0];
            String taskPath = domain.getValue()[1];
            //Path domainFilePath = Paths.get(domainPath).toAbsolutePath();

            try(Stream<java.nio.file.Path> problems = Files.list(Paths.get(taskPath))){
                problems.filter(p -> p.toString().endsWith(".pddl") && !p.getFileName().toString().equals("domain.pddl"))
                    .sorted()
                    .forEach(problemFile ->{
                        
                        try{
                            String problemName = problemFile.getFileName().toString().replace(".pddl","");

                            //Excecute planner for MCTS
                            String mctsPerformance = runPlanner(planner,domainPath,problemFile.toString());

                            //Execute planner for HSP
                            String hspPerformance = runPlanner(hspPlanner,domainPath, problemFile.toString());

                            //Write to csv file
                            csvFile.append(String.format("%s,%s,%s,%s \n", domainName,problemName,mctsPerformance,hspPerformance));

                            LOGGER.info("Successfully completed comparison: {} - {}",domainName, problemName);

                        }
                        catch(Exception e){
                            LOGGER.error("Could not process {}: {}",problemFile, e.getMessage());
                        }
                    });
            }
        }
        Files.write(Paths.get("mcts_vs_hsp_comparison.csv"), csvFile.toString().getBytes());
        LOGGER.info("Comparative evaluation complete. Results: mcts_vs_hsp_comparison.csv");
    }

    private static String runPlanner(AbstractPlanner planner, String domainFile, String problemFile){
        try{
            planner.setDomain(domainFile);
            planner.setProblem(problemFile);

            Plan plan = planner.solve();
            double totalTime = planner.getStatistics().getTimeToParse() + planner.getStatistics().getTimeToEncode() + planner.getStatistics().getTimeToSearch();

            int planLength = (plan != null) ? plan.size() : 0;
            return String.format("%.2f,%d", totalTime, planLength);

        }
        catch(Exception e){
            return "Error,0";
        }
    }

    

    /** 
     * The following is the main method of the MCTS planner.
     * @param arguments The parameters are the command line arguments. 
     */
    public static void main(String[] args){
        try{
            comparePerformance();

        } 
        catch(IllegalArgumentException e){
            LOGGER.fatal(e.getMessage());
        }
        catch(IOException e){
            LOGGER.fatal("Comparison failed" + e.getMessage());
        }
    }
}

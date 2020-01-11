using Test

import ConstraintSolver
const CS = ConstraintSolver
import MathOptInterface
const MOI = MathOptInterface

# max cardinality stable set problem
# given G = (V, E)
# max sum_{i in V}(x_i)
# s.t.
# x_i + x_j <= 1 ∀ (i, j) ∈ E
# x_i ∈ {0,1}
@testset "Stable set" begin
    matrix = [
        0 1 1 0
        1 0 1 0
        1 1 0 1
        0 0 1 0
    ]
    model = CS.ConstraintSolverModel()
    x = [ConstraintSolver.add_var!(model, 0, 1) for _ in 1:4]
    for i in 1:4, j in 1:4
        if matrix[i,j] == 1
            # ConstraintSolver.add_constraint!(model, x[i] + x[j] <= 1)
        end
    end
end

@testset "Stable set MOI" begin
    matrix = [
        0 1 1 0
        1 0 1 0
        1 1 0 1
        0 0 1 0
    ]
    model = CS.Optimizer()
    x = [MOI.add_constrained_variable(model, MOI.ZeroOne()) for _ in 1:4]
    for i in 1:4, j in 1:4
        if matrix[i,j] == 1 && i < j
            (z, _) = MOI.add_constrained_variable(model, MOI.GreaterThan(0.0))
            MOI.add_constraint(model, z, MOI.Integer())
            MOI.add_constraint(model, z, MOI.LessThan(1.0))
            f = MOI.ScalarAffineFunction(
                [
                    MOI.ScalarAffineTerm(1.0, x[i][1]),
                    MOI.ScalarAffineTerm(1.0, x[j][1]),
                    MOI.ScalarAffineTerm(1.0, z),
                ], 0.0
            )
            MOI.add_constraint(model, f, MOI.EqualTo(1.0))
            # ConstraintSolver.add_constraint!(model, x[i] + x[j] <= 1)
        end
    end
    # sum(x) - stable_set == 0
    stable_set = MOI.add_variable(model)
    MOI.add_constraint(model, stable_set, MOI.Integer())
    MOI.add_constraint(model, stable_set, MOI.LessThan(4.0))
    MOI.add_constraint(model, stable_set, MOI.GreaterThan(0.0))
    terms = [MOI.ScalarAffineTerm(1.0, xi[1]) for xi in x]
    push!(terms, MOI.ScalarAffineTerm(-1.0, stable_set))
    MOI.add_constraint(model,
        MOI.ScalarAffineFunction(terms, 0.0),
        MOI.EqualTo(0.0),
    )
    MOI.set(model, MOI.ObjectiveFunction{MOI.SingleVariable}(), MOI.SingleVariable(stable_set))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    MOI.optimize!(model)
    @test MOI.get(model, MOI.VariablePrimal(), x[4][1]) == 1
    @test MOI.get(model, MOI.VariablePrimal(), x[1][1]) + MOI.get(model, MOI.VariablePrimal(), x[2][1]) == 1
    @test MOI.get(model, MOI.ObjectiveValue()) == 2
end

@testset "Weighted stable set" begin
    matrix = [
        0 1 1 0
        1 0 1 0
        1 1 0 1
        0 0 1 0
    ]
    model = CS.Optimizer()
    x = [MOI.add_constrained_variable(model, MOI.ZeroOne()) for _ in 1:4]
    for i in 1:4, j in 1:4
        if matrix[i,j] == 1 && i < j
            (z, _) = MOI.add_constrained_variable(model, MOI.GreaterThan(0.0))
            MOI.add_constraint(model, z, MOI.Integer())
            MOI.add_constraint(model, z, MOI.LessThan(1.0))
            f = MOI.ScalarAffineFunction(
                [
                    MOI.ScalarAffineTerm(1.0, x[i][1]),
                    MOI.ScalarAffineTerm(1.0, x[j][1]),
                    MOI.ScalarAffineTerm(1.0, z),
                ], 0.0
            )
            MOI.add_constraint(model, f, MOI.EqualTo(1.0))
            # ConstraintSolver.add_constraint!(model, x[i] + x[j] <= 1)
        end
    end
    # sum(w[i] * x[i] for i in V) - stable_set == 0
    stable_set = MOI.add_variable(model)
    MOI.add_constraint(model, stable_set, MOI.Integer())
    MOI.add_constraint(model, stable_set, MOI.LessThan(4.0))
    MOI.add_constraint(model, stable_set, MOI.GreaterThan(0.0))
    weights = [2.0, 1.0, 2.0, 1.0]
    terms = [MOI.ScalarAffineTerm(weights[i], x[i][1]) for i in eachindex(x)]
    push!(terms, MOI.ScalarAffineTerm(-1.0, stable_set))
    MOI.add_constraint(model,
        MOI.ScalarAffineFunction(terms, 0.0),
        MOI.EqualTo(0.0),
    )
    MOI.set(model, MOI.ObjectiveFunction{MOI.SingleVariable}(), MOI.SingleVariable(stable_set))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    MOI.optimize!(model)
    @test MOI.get(model, MOI.VariablePrimal(), x[4][1]) == 1
    @test MOI.get(model, MOI.VariablePrimal(), x[1][1]) == 1
    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 3
end

#pragma once

#include "core/tracker/ekf.hpp"
#include <Eigen/Eigen>
#include <utility>
#include <vector>

class IMM {
public:
    IMM() = default;

    IMM(const std::vector<EKF>& models, Eigen::MatrixXd transitionProbabilities)
        : models_(models)
        , transitionProbabilities_(std::move(transitionProbabilities)) {
        modeProbabilities_ = Eigen::VectorXd::Constant(
            static_cast<Eigen::Index>(models.size()), 1.0 / (double)models.size());
        mixedStates_.resize(static_cast<Eigen::Index>(models.size()));
        mixedCovariances_.resize(static_cast<Eigen::Index>(models.size()));
    }

    void predict(const double& u) {
        mixProbabilities();
        for (auto& model : models_) {
            model.Predict(u);
        }
    }

    Eigen::VectorXd predictSecond(const double& u) const {
        Eigen::VectorXd state, result;
        for (auto& model : models_) {
            state << model.PredictConst(u);
        }

        result.setZero(models_[0].x_.size());

        for (size_t i = 0; i < models_.size(); ++i) {
            result[static_cast<Eigen::Index>(i)] += modeProbabilities_[static_cast<Eigen::Index>(i)]
                                                  * state[static_cast<Eigen::Index>(i)];
        }
        return result;
    }

    void update(const Eigen::VectorXd& z) {
        Eigen::VectorXd likelihoods(models_.size());

        for (size_t i = 0; i < models_.size(); ++i) {
            models_[i].Update(z);
            likelihoods[static_cast<Eigen::Index>(i)] = computeLikelihood(models_[i], z);
        }

        updateModeProbabilities(likelihoods);
        combineEstimates();
    }

    Eigen::VectorXd& getState() { return state_; }
    const Eigen::MatrixXd& getCovariance() const { return covariance_; }

private:
    std::vector<EKF> models_;
    Eigen::MatrixXd transitionProbabilities_;
    Eigen::VectorXd modeProbabilities_;
    std::vector<Eigen::VectorXd> mixedStates_;
    std::vector<Eigen::MatrixXd> mixedCovariances_;
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;

    void mixProbabilities() {
        size_t n = models_.size();
        Eigen::MatrixXd mixedProbs =
            transitionProbabilities_.transpose() * modeProbabilities_.asDiagonal();

        for (size_t i = 0; i < n; ++i) {
            modeProbabilities_[static_cast<Eigen::Index>(i)] =
                mixedProbs.col(static_cast<Eigen::Index>(i)).sum();
        }

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                mixedStates_[i] += transitionProbabilities_(
                                       static_cast<Eigen::Index>(j), static_cast<Eigen::Index>(i))
                                 * modeProbabilities_[j] * models_[j].x_;
            }
        }
    }

    static double computeLikelihood(const EKF& model, const Eigen::VectorXd& z) {
        Eigen::VectorXd residual = z - model.x_;
        Eigen::MatrixXd S        = model.P_;
        double det               = S.determinant();
        double normFactor        = std::sqrt(std::pow(2 * M_PI, z.size()) * det);
        double likelihood =
            std::exp(-0.5 * residual.transpose() * S.inverse() * residual) / normFactor;
        return likelihood;
    }

    void updateModeProbabilities(const Eigen::VectorXd& likelihoods) {
        modeProbabilities_ = (modeProbabilities_.array() * likelihoods.array()).matrix();
        modeProbabilities_ /= modeProbabilities_.sum();
    }

    void combineEstimates() {
        state_.setZero(models_[0].x_.size());
        covariance_.setZero(models_[0].P_.rows(), models_[0].P_.cols());

        for (size_t i = 0; i < models_.size(); ++i) {
            state_ += modeProbabilities_[static_cast<Eigen::Index>(i)] * models_[i].x_;
            covariance_ += modeProbabilities_[static_cast<Eigen::Index>(i)] * models_[i].P_;
        }
    }
};

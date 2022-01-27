package com.example.maccproject.ui.quiz

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import androidx.navigation.findNavController
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentQuizBinding

class QuizFragment : Fragment() {

    private lateinit var quizViewModel: QuizViewModel
    private var _binding: FragmentQuizBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle? ): View {
        quizViewModel =
            ViewModelProvider(this).get(QuizViewModel::class.java)

        _binding = FragmentQuizBinding.inflate(inflater, container, false)
        val root: View = binding.root

        val textQuizTitle: TextView = binding.textQuiz
        quizViewModel.quizTitle.observe(viewLifecycleOwner, {
            textQuizTitle.text = it
        })
        val textQuizGeneralInfo: TextView = binding.quizGeneralInfo
        quizViewModel.quizGeneralInfo.observe(viewLifecycleOwner, {
            textQuizGeneralInfo.text = it
        })

        binding.playButton.setOnClickListener { v: View ->
            v.findNavController().navigate(R.id.action_navigation_quiz_to_quiz_graphic)
        }

        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}